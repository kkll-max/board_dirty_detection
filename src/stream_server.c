/*
 * stream_server.c — 摄像头取流 + MJPEG 网页预览（纯 C11）。
 * 加 --model 时顺带做逐帧语义分割（镜头脏污检测），脏污区域直接叠加在
 * 推流画面上；不开 --model 则和原来一样只推流。
 *
 * 两种帧源：
 *
 *   --watch DIR   监听目录里新出现的 PNG（inotify IN_CLOSE_WRITE），解码后转
 *                 JPEG 推流。配合板端 /goko/app/sub_img_imu 使用：
 *                     sub_img_imu 2 &
 *                     ./stream_server --watch /userdata/mono_orig/image_data/stereo_left
 *                 这条路不碰相机、不依赖任何 SDK，现在就能跑。
 *
 *   --camera      通过 Metoak MoGeneralSDK 的 C API 直接打开 S316 双目模组，
 *                 取 640x544 yuv420p 左右目，转 RGB→JPEG 推流。SDK 用
 *                 dlopen 在运行时加载（因此本程序没有硬依赖，--watch 模式在
 *                 任何板子上都能跑）。注意：sensor 进程也在用同一个模组，
 *                 直连可能与它冲突（表现为 moOpenCamera 失败或需先停 sensor）。
 *
 * HTTP 接口：
 *   GET /               HTML 预览页（<img src="/stream.mjpg">）
 *   GET /stream.mjpg    multipart/x-mixed-replace MJPEG 流（多客户端）
 *   GET /status.json    {seq,width,height,pub_width,fps,frames,jpeg_bytes,source,
 *                        infer,dirty,cls_ratio,fg_ratio,infer_ms,
 *                        n_region,regions:[{cls,x,y,w,h,area},...]}
 *   GET /snapshot.jpg   最新一帧 JPEG
 *   GET /infer?on=0|1   网页开关：实时推理并叠加 / 仅推流
 *   GET /res?w=N        网页切换推流宽度：0 = 原始尺寸，否则等比缩到 N 宽
 *
 * 数据采集（网页上的“保存 / 保存至 / 新建”，全部限定在
 * /userdata/iris/data_get 下）：
 *   GET /save                 保存当前帧（叠加前的原始 RGB，JPEG q95）
 *   GET /dirs.json?path=rel   列出 data_get 下的子目录
 *   GET /setdir?path=rel      切换保存目录（持久化到 data_get/.selected）
 *   GET /mkdir?parent=rel&name=xxx   在 data_get 下新建子目录
 *
 * 编译（板端 aarch64 用 Makefile 或 scripts/build.sh；x86 直接 gcc 即可）：
 *   gcc -O2 -std=c11 -Wall -Wextra -pthread -Ithird_party \
 *       -o stream_server stream_server.c -lm -ldl
 */

#define _GNU_SOURCE

#include <arpa/inet.h>
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/inotify.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* STBI_ONLY_PNG 会让 stb_image 里 JPEG/GIF 专用的静态函数变成“未使用” */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#pragma GCC diagnostic pop

/* 叠字用 stb_easy_font（与 infer_board.c 的可视化一致） */
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-braces"
#endif
#include "stb_easy_font.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include "hb_dnn.h"

#define PROG "stream_server"
#define DEFAULT_PORT 8080
#define DEFAULT_FPS 10
#define DEFAULT_QUALITY 70
#define DEFAULT_PUB_WIDTH 0        /* 推流宽度上限，0 = 原始尺寸 */
#define DEFAULT_TARGET_CLASS 1   /* 1 = labels.txt 里的第一个脏污类 */
#define DEFAULT_DIRTY_RATIO 0.5   /* 所有脏污类合计面积占比 %% */
#define SEG_MAX_CLASSES 16        /* labels.txt 最多支持 16 类（0 = clean） */
#define SEG_CLASS_NAME_MAX 32
#define DEFAULT_SAVE_DIR "/userdata/iris/dirty_imgs"
#define DEFAULT_SAVE_MAX 100      /* 目录内最多保留张数，0 = 不保存 */
/* 两张脏污对比图的最小间隔（ms）：0 = 每脏污帧都存。三联图 JPEG 编码 +
 * eMMC 写入约 50~80 ms，逐帧保存会把推流卡到 5 fps 以下、网页延迟变大。 */
#define DEFAULT_SAVE_INTERVAL_MS 1000
/* 脏污连通域：在 640x384 类别图上做 8 邻域连通域分析，面积小于
 * SEG_REGION_MIN_PX（模型像素）的当噪点丢掉；状态接口/网页最多给出
 * SEG_MAX_REGIONS 个（按面积从大到小）。 */
#define SEG_MAX_REGIONS 16
#define SEG_REGION_MIN_PX 16
#define DEFAULT_RECORD_DIR "/userdata/iris/on_time"
#define DEFAULT_RECORD_MAX 100    /* -a/-b 默认最多保留的帧数 */
#define RECORD_MIN_FREE_MB 200.0  /* 剩余空间低于此值自动停止录制 */
#define MAX_CLIENTS 8

/* labels.txt 的类名（启动时加载一次，之后只读；/labels.json 和网页图例用） */
static int g_n_class;
static char g_cls_name[SEG_MAX_CLASSES][SEG_CLASS_NAME_MAX];

/* 推流画面宽度上限：启动时由 --pub-width 设定，网页 /res?w=N 可随时改。
 * 0 = 原始尺寸；watch 线程每帧原子读取，改变后下一帧立即生效。 */
static int g_pub_width = DEFAULT_PUB_WIDTH;

static int pub_width_set(int w)
{
    if (w < 0)
        w = 0; /* 负数按“不缩”处理 */
    else if (w > 0 && w < 64)
        w = 64; /* 再小就缩成细条了，没有意义 */
    __atomic_store_n(&g_pub_width, w, __ATOMIC_RELAXED);
    return w;
}

/* 每类叠加颜色（RGB）：0 clean 不叠加；1 dirt_dot 红褐、2 glare 明黄、
 * 3 water 蓝、4 grass_residue 绿、5 mud_film 黄褐，其余类按表取色 */
static const uint8_t SEG_PALETTE_RGB[SEG_MAX_CLASSES][3] = {
    {0, 0, 0},       {165, 42, 42},   {255, 255, 0},   {0, 0, 255},
    {0, 200, 0},     {180, 120, 40},  {0, 200, 200},   {255, 128, 0},
    {140, 90, 255},  {255, 255, 255}, {128, 128, 128}, {90, 160, 90},
    {200, 160, 120}, {60, 120, 200},  {180, 180, 60},  {120, 60, 160},
};

/* ---------------------------------------------------------------- 基础工具 */

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

static void logmsg(const char *fmt, ...)
{
    struct timespec ts;
    struct tm tm;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm);
    fprintf(stderr, "%04d-%02d-%02d %02d:%02d:%02d.%03ld ",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000L);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static int str_ends_with_ci(const char *s, const char *suffix)
{
    size_t ns = strlen(s), nf = strlen(suffix);
    if (nf > ns)
        return 0;
    return strcasecmp(s + ns - nf, suffix) == 0;
}

static int has_png_ext(const char *name)
{
    return str_ends_with_ci(name, ".png");
}

/* 可增长缓冲区：stb 的 JPEG 编码经由回调写入 */
typedef struct {
    unsigned char *data;
    size_t len, cap;
    int err;
} Buffer;

static int buf_reserve(Buffer *b, size_t need)
{
    if (b->cap >= need)
        return 0;
    size_t cap = b->cap ? b->cap : 64 * 1024;
    while (cap < need)
        cap *= 2;
    unsigned char *p = realloc(b->data, cap);
    if (!p) {
        b->err = 1;
        return -1;
    }
    b->data = p;
    b->cap = cap;
    return 0;
}

static void jpeg_sink(void *ctx, void *data, int size)
{
    Buffer *b = (Buffer *)ctx;
    if (size <= 0 || b->err)
        return;
    if (buf_reserve(b, b->len + (size_t)size) != 0)
        return;
    memcpy(b->data + b->len, data, (size_t)size);
    b->len += (size_t)size;
}

static int encode_jpeg(const uint8_t *rgb, int w, int h, int quality, Buffer *out)
{
    out->len = 0;
    out->err = 0;
    if (quality < 1)
        quality = 1;
    if (quality > 100)
        quality = 100;
    if (!stbi_write_jpg_to_func(jpeg_sink, out, w, h, 3, rgb, quality))
        return -1;
    return out->err ? -1 : 0;
}

/* ------------------------------------------------------------ 帧状态中心 */

/* 脏污连通域：坐标已映射回原始帧像素（x/y 为包围盒左上角，w/h 为宽高），
 * area 是连通域在模型网格上的像素数（640x384 坐标）。 */
typedef struct {
    int cls;
    int x, y, w, h;
    long area;
} SegRegion;

/* 本帧的分割结果（--model 开启时由 process_png 填充） */
typedef struct {
    int enabled;        /* 1 = 这张图跑了模型 */
    int dirty;          /* 1 = 所有脏污类（class>=1）合计面积超过阈值 */
    int n_class;
    int target_class;
    long target_px;
    double target_ratio; /* 0..100，目标类像素占比 */
    double fg_ratio;     /* 0..100，class1..n-1 合计占比 */
    double cls_ratio[SEG_MAX_CLASSES]; /* 各类像素占比 0..100 */
    double infer_ms;
    int n_region;                          /* 连通域个数（最多 SEG_MAX_REGIONS） */
    SegRegion region[SEG_MAX_REGIONS];     /* 按面积从大到小 */
} InferInfo;

typedef struct {
    pthread_mutex_t mu;
    pthread_cond_t cv;
    unsigned char *jpeg;        /* 最新一帧 JPEG */
    size_t jpeg_len;
    unsigned char *raw;         /* 最新一帧原始 RGB（叠加前），数据采集用 */
    size_t raw_len;
    int raw_w, raw_h;
    unsigned long seq;          /* 已发布帧序号（从 1 开始） */
    int width, height;
    unsigned long frames;       /* 累计发布帧数 */
    double fps;                 /* 最近 1s 的发布帧率 */
    double win_start;
    unsigned long win_frames;
    double last_pub_ms;
    double last_log;
    char source[256];           /* 帧源描述（文件名 / camera:left） */
    int infer;                  /* 本帧是否跑了模型 */
    int has_model;              /* 启动时是否加载了模型（决定“是否开启”按钮可用） */
    int infer_on;               /* 网页开关：1 = 实时推理并叠加，0 = 仅推流 */
    int dirty;                  /* 镜头是否脏污 */
    int n_class;
    int target_class;
    double target_ratio;        /* 目标类像素占比 0..100 */
    double fg_ratio;            /* 所有脏污类合计占比 0..100 */
    double cls_ratio[SEG_MAX_CLASSES];
    double infer_ms;            /* 单帧推理（含预处理）耗时 */
    int n_region;               /* 脏污连通域个数 */
    SegRegion region[SEG_MAX_REGIONS];
    int quit;
} FrameHub;

static void hub_init(FrameHub *h)
{
    memset(h, 0, sizeof(*h));
    pthread_mutex_init(&h->mu, NULL);
    pthread_cond_init(&h->cv, NULL);
}

static void hub_destroy(FrameHub *h)
{
    free(h->jpeg);
    h->jpeg = NULL;
    free(h->raw);
    h->raw = NULL;
    pthread_cond_destroy(&h->cv);
    pthread_mutex_destroy(&h->mu);
}

/* 数据采集：留存最新一帧未叠加的 RGB（调用方在 seg_process 改写 rgb 之前调用） */
static void hub_set_raw(FrameHub *h, const unsigned char *rgb, int w, int hh)
{
    if (h->quit || !rgb || w <= 0 || hh <= 0)
        return;
    const size_t need = (size_t)w * (size_t)hh * 3;
    pthread_mutex_lock(&h->mu);
    if (h->raw_len != need) {
        unsigned char *p = realloc(h->raw, need);
        if (!p) {
            pthread_mutex_unlock(&h->mu);
            return;
        }
        h->raw = p;
        h->raw_len = need;
    }
    memcpy(h->raw, rgb, need);
    h->raw_w = w;
    h->raw_h = hh;
    pthread_mutex_unlock(&h->mu);
}

static void hub_publish(FrameHub *h, const unsigned char *jpeg, size_t len,
                        int w, int hh, const char *src, const InferInfo *inf)
{
    if (h->quit || len == 0)
        return;
    double t = now_ms();
    unsigned char *copy = malloc(len);
    if (!copy) {
        logmsg("内存不足，丢弃一帧 (%zu 字节)", len);
        return;
    }
    memcpy(copy, jpeg, len);

    pthread_mutex_lock(&h->mu);
    free(h->jpeg);
    h->jpeg = copy;
    h->jpeg_len = len;
    h->width = w;
    h->height = hh;
    h->seq++;
    h->frames++;
    snprintf(h->source, sizeof(h->source), "%s", src);
    h->last_pub_ms = t;
    if (inf && inf->enabled) {
        h->infer = 1;
        h->dirty = inf->dirty;
        h->n_class = inf->n_class;
        h->target_class = inf->target_class;
        h->target_ratio = inf->target_ratio;
        h->fg_ratio = inf->fg_ratio;
        memcpy(h->cls_ratio, inf->cls_ratio, sizeof(h->cls_ratio));
        h->infer_ms = inf->infer_ms;
        h->n_region = inf->n_region;
        memcpy(h->region, inf->region, sizeof(h->region));
    } else {
        h->infer = 0;
        h->dirty = 0;
        h->target_ratio = 0;
        h->fg_ratio = 0;
        memset(h->cls_ratio, 0, sizeof(h->cls_ratio));
        h->infer_ms = 0;
        h->n_region = 0;
    }
    h->win_frames++;
    if (h->win_start <= 0)
        h->win_start = t;
    if (t - h->win_start >= 1000.0) {
        h->fps = (double)h->win_frames * 1000.0 / (t - h->win_start);
        h->win_start = t;
        h->win_frames = 0;
    }
    unsigned long frames = h->frames;
    double fps = h->fps;
    pthread_mutex_unlock(&h->mu);
    pthread_cond_broadcast(&h->cv);

    if (t - h->last_log >= 10000.0) {
        h->last_log = t;
        logmsg("已推 %lu 帧, %.1f fps, 最新 %dx%d %zu B [%s]",
               frames, fps, w, hh, len, src);
    }
}

/* ------------------------------------------------------------------ HTTP */

static int send_all(int fd, const void *buf, size_t len)
{
    const char *p = (const char *)buf;
    while (len > 0) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (n == 0)
            return -1;
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static int send_http(int fd, int code, const char *reason, const char *ctype,
                     const void *body, size_t len)
{
    char hdr[512];
    int n = snprintf(hdr, sizeof(hdr),
                     "HTTP/1.0 %d %s\r\n"
                     "Content-Type: %s\r\n"
                     "Content-Length: %zu\r\n"
                     "Cache-Control: no-store\r\n"
                     "Connection: close\r\n"
                     "\r\n",
                     code, reason, ctype, len);
    if (n < 0 || (size_t)n >= sizeof(hdr))
        return -1;
    if (send_all(fd, hdr, (size_t)n) != 0)
        return -1;
    if (len && send_all(fd, body, len) != 0)
        return -1;
    return 0;
}

static const char PAGE_HTML[] =
    "<!doctype html><html lang=\"zh\"><head><meta charset=\"utf-8\">\n"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
    "<title>goko camera</title>\n"
    "<style>\n"
    "body{margin:0;background:#101214;color:#e6e6e6;font:14px/1.5 system-ui,"
    "-apple-system,Segoe UI,sans-serif;display:flex;flex-direction:column;"
    "align-items:center}\n"
    "header{width:100%;box-sizing:border-box;padding:10px 16px;display:flex;"
    "gap:16px;align-items:baseline;border-bottom:1px solid #262a2f}\n"
    "header b{font-size:15px;color:#fff}\n"
    "#st{color:#9aa4ad;font-variant-numeric:tabular-nums}\n"
    ".wrap{margin:12px;position:relative;max-width:100%}\n"
    "img{display:block;max-width:100%;height:auto;border-radius:6px;"
    "background:#000}\n"
    "#err{position:absolute;inset:0;display:none;align-items:center;"
    "justify-content:center;color:#fc6;background:#000a;border-radius:6px}\n"
    ".tbar{width:100%;box-sizing:border-box;padding:8px 16px;display:flex;"
    "gap:8px;align-items:center;flex-wrap:wrap;background:#15181b;"
    "border-bottom:1px solid #262a2f}\n"
    ".tbar button,.dlg button{cursor:pointer;background:#22272c;color:#e6e6e6;"
    "border:1px solid #3a4046;border-radius:6px;padding:6px 14px;font:inherit}\n"
    ".tbar button:hover,.dlg button:hover{background:#2c3238}\n"
    ".tbar button:disabled{opacity:.5;cursor:default}\n"
    "#saveS{background:#2563eb;border-color:#2563eb;color:#fff}\n"
    "#saveS:hover{background:#1d4ed8}\n"
    "#cur{color:#9aa4ad;margin-left:auto;font-family:ui-monospace,monospace;"
    "font-size:12px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;"
    "max-width:62%}\n"
    "#toast{position:fixed;left:50%;bottom:24px;transform:translateX(-50%);"
    "background:#1f2937e6;color:#fff;padding:8px 16px;border-radius:8px;"
    "opacity:0;transition:opacity .2s;pointer-events:none;max-width:80vw;"
    "z-index:20}\n"
    "#toast.on{opacity:1}\n"
    "#modal{position:fixed;inset:0;display:none;align-items:center;"
    "justify-content:center;background:#000a;z-index:10}\n"
    "#modal.on{display:flex}\n"
    ".dlg{background:#181c20;border:1px solid #30363c;border-radius:10px;"
    "width:min(520px,92vw);max-height:80vh;display:flex;flex-direction:column;"
    "overflow:hidden}\n"
    ".dlg h3{margin:0;padding:12px 16px;font-size:14px;color:#fff;"
    "border-bottom:1px solid #262a2f}\n"
    "#dpath{padding:8px 16px;color:#9aa4ad;font-family:ui-monospace,monospace;"
    "font-size:12px;word-break:break-all}\n"
    "#dlist{overflow:auto;flex:1;min-height:120px}\n"
    "#dlist div{padding:8px 16px;cursor:pointer;border-top:1px solid #22262b}\n"
    "#dlist div:hover{background:#22272c}\n"
    "#dempty{padding:16px;color:#6b7480}\n"
    ".dlg .foot{display:flex;gap:8px;padding:12px 16px;flex-wrap:wrap;"
    "border-top:1px solid #262a2f}\n"
    ".dlg .foot .sp{flex:1}\n"
    "#pickS{background:#2563eb;border-color:#2563eb;color:#fff}\n"
    "#infS.on{background:#15803d;border-color:#15803d;color:#fff}\n"
    "#infS.on:hover{background:#166534}\n"
    "#infS:disabled{opacity:.5;cursor:default}\n"
    ".res{color:#9aa4ad;font-size:13px;white-space:nowrap}\n"
    "#resS{background:#22272c;color:#e6e6e6;border:1px solid #3a4046;"
    "border-radius:6px;padding:6px 8px;font:inherit;margin-left:6px}\n"
    ".sw{display:inline-block;width:10px;height:10px;border-radius:2px;"
    "margin-right:4px;vertical-align:-1px}\n"
    "#lbar{font-size:12px;color:#9aa4ad}\n"
    "</style></head><body>\n"
    "<header><b>goko camera</b><span id=\"st\">等待画面…</span></header>\n"
    "<div class=\"tbar\"><button id=\"saveS\" onclick=\"saveNow()\">保存</button>"
    "<button onclick=\"openPick()\">保存至</button>"
    "<button onclick=\"newDir()\">新建</button>"
    "<button id=\"infS\" onclick=\"toggleInfer()\" disabled>"
    "是否开启脏污检测: --</button>"
    "<label class=\"res\">推流宽度<select id=\"resS\""
    " onchange=\"setRes(this.value)\">"
    "<option value=\"0\">原始</option><option value=\"1282\">1282</option>"
    "<option value=\"960\">960</option><option value=\"640\">640</option>"
    "<option value=\"480\">480</option><option value=\"320\">320</option>"
    "</select></label>"
    "<span id=\"cur\">保存至: data_get/</span></div>\n"
    "<div class=\"wrap\"><img id=\"v\" src=\"/stream.mjpg\" alt=\"camera\">"
    "<div id=\"err\">连接中断，重试中…</div></div>\n"
    "<div class=\"tbar\" id=\"lbar\" style=\"display:none\"></div>\n"
    "<div id=\"toast\"></div>\n"
    "<div id=\"modal\"><div class=\"dlg\"><h3>选择保存位置</h3>"
    "<div id=\"dpath\">data_get/</div><div id=\"dlist\"></div>"
    "<div class=\"foot\"><button onclick=\"upDir()\">上一级</button>"
    "<button onclick=\"newDir()\">新建</button><span class=\"sp\"></span>"
    "<button id=\"pickS\" onclick=\"pickDir()\">选择此文件夹</button>"
    "<button onclick=\"closePick()\">取消</button></div></div></div>\n"
    "<script>\n"
    "var img=document.getElementById('v'),st=document.getElementById('st'),"
    "err=document.getElementById('err');\n"
    "var cur='',browse='',modal=document.getElementById('modal'),"
    "dlist=document.getElementById('dlist'),dpath=document.getElementById('dpath'),"
    "curL=document.getElementById('cur');\n"
    "var infS=document.getElementById('infS'),inferOn=false,hasModel=false,"
    "labels=null,nmEls=[];\n"
    "function reconnect(){err.style.display='flex';"
    "setTimeout(function(){img.src='/stream.mjpg?t='+Date.now();},500);}\n"
    "img.onerror=reconnect;\n"
    "img.onload=function(){err.style.display='none';};\n"
    "setInterval(function(){fetch('/status.json',{cache:'no-store'})"
    ".then(function(r){return r.json();}).then(function(j){"
    "st.textContent=j.width+'x'+j.height+'  '+j.fps.toFixed(1)+' fps  帧 '"
    "+j.seq+(j.infer_on?('  脏污:'+(j.dirty?('是 '+j.fg_ratio.toFixed(1)+'%')"
    ":'否')):'')+(j.infer_on?('  区域 '+(j.n_region||0)):'')+'  '+j.source;"
    "hasModel=!!j.has_model;inferOn=!!j.infer_on;updateInfBtn();"
    "updateRatios(j);syncRes(j.pub_width);"
    "if(j.seq>0)err.style.display='none';}).catch(function(){});},500);\n"
    "function updateInfBtn(){if(!hasModel){infS.disabled=true;"
    "infS.textContent='是否开启脏污检测: 无模型';infS.classList.remove('on');"
    "return;}infS.disabled=false;infS.textContent='是否开启脏污检测: '"
    "+(inferOn?'开':'关');infS.classList.toggle('on',inferOn);}\n"
    "function toggleInfer(){if(!hasModel)return;var want=inferOn?0:1;"
    "infS.disabled=true;fetch('/infer?on='+want,{cache:'no-store'})"
    ".then(function(r){return r.json();}).then(function(j){"
    "if(!j.ok){toast('切换失败: '+(j.error||'不可用'));return;}"
    "inferOn=!!j.on;toast('脏污检测已'+(inferOn?'开启':'关闭'));})"
    ".catch(function(){toast('切换失败: 网络错误');})"
    ".then(function(){updateInfBtn();});}\n"
    "var resS=document.getElementById('resS');\n"
    "function syncRes(w){if(w==null)return;var v=String(w),f=false;"
    "for(var i=0;i<resS.options.length;i++)"
    "if(resS.options[i].value===v){f=true;break;}"
    "if(!f){var o=document.createElement('option');o.value=v;"
    "o.textContent=v+' 宽';resS.appendChild(o);}"
    "if(document.activeElement!==resS)resS.value=v;}\n"
    "function setRes(v){resS.disabled=true;"
    "fetch('/res?w='+v,{cache:'no-store'})"
    ".then(function(r){return r.json();}).then(function(j){"
    "if(!j.ok){toast('切换分辨率失败: '+(j.error||'不可用'));return;}"
    "toast(j.w>0?('推流宽度 '+j.w+'（等比缩放）'):'原始分辨率（不缩放）');})"
    ".catch(function(){toast('切换分辨率失败: 网络错误');})"
    ".then(function(){resS.disabled=false;});}\n"
    "function updateRatios(j){if(!nmEls.length||!j.cls_ratio)return;"
    "for(var i=0;i<nmEls.length;i++){var v=j.cls_ratio[i];"
    "nmEls[i].textContent=labels[i]+(v==null?'':' '+v.toFixed(1)+'%');}}\n"
    "function toast(m){var t=document.getElementById('toast');"
    "t.textContent=m;t.classList.add('on');clearTimeout(t._h);"
    "t._h=setTimeout(function(){t.classList.remove('on');},2400);}\n"
    "function showCur(){curL.textContent='保存至: data_get/'+(cur||'');}\n"
    "function saveNow(){var b=document.getElementById('saveS');b.disabled=true;"
    "fetch('/save',{cache:'no-store'}).then(function(r){return r.json();})"
    ".then(function(j){if(j.ok){toast('已保存 '+j.file+'  → data_get/'"
    "+((j.dir||'')?j.dir+'/':''));}else{toast('保存失败: '+j.error);}})"
    ".catch(function(){toast('保存失败: 网络错误');})"
    ".then(function(){b.disabled=false;});}\n"
    "function render(j){dpath.textContent='data_get/'+(j.path||'');"
    "dlist.innerHTML='';var ds=j.dirs||[];"
    "if(!ds.length){var e=document.createElement('div');e.id='dempty';"
    "e.textContent='（没有子文件夹）';dlist.appendChild(e);return;}"
    "ds.forEach(function(n){var d=document.createElement('div');"
    "d.textContent='📁 '+n;d.onclick=function(){loadDirs("
    "((j.path||'')?j.path+'/':'')+n);};dlist.appendChild(d);});}\n"
    "function loadDirs(p){fetch('/dirs.json?path='+encodeURIComponent(p||''),"
    "{cache:'no-store'}).then(function(r){return r.json();})"
    ".then(function(j){if(!j.ok){if(p){loadDirs('');return;}"
    "toast('读取目录失败: '+j.error);return;}"
    "browse=j.path||'';cur=j.cur||'';showCur();render(j);})"
    ".catch(function(){toast('读取目录失败: 网络错误');});}\n"
    "function openPick(){modal.classList.add('on');loadDirs(cur);}\n"
    "function closePick(){modal.classList.remove('on');}\n"
    "function upDir(){if(!browse)return;var i=browse.lastIndexOf('/');"
    "loadDirs(i<0?'':browse.slice(0,i));}\n"
    "function pickDir(){fetch('/setdir?path='+encodeURIComponent(browse),"
    "{cache:'no-store'}).then(function(r){return r.json();})"
    ".then(function(j){if(!j.ok){toast('切换失败: '+j.error);return;}"
    "cur=j.cur||'';showCur();closePick();"
    "toast('保存位置: data_get/'+(cur?cur+'/':'')+'');})"
    ".catch(function(){toast('切换失败: 网络错误');});}\n"
    "function newDir(){var inModal=modal.classList.contains('on');"
    "var parent=inModal?(browse||''):(cur||'');"
    "var where='（在 data_get/'+parent+(parent?'/':'')+' 下）';"
    "var n=prompt('新建文件夹名称'+where);if(!n)return;"
    "n=n.trim();if(!n){toast('名称不能为空');return;}"
    "fetch('/mkdir?parent='+encodeURIComponent(parent)+'&name='"
    "+encodeURIComponent(n),{cache:'no-store'})"
    ".then(function(r){return r.json();}).then(function(j){"
    "if(!j.ok){toast('新建失败: '+j.error);return;}"
    "toast('已新建 '+j.name);if(inModal)loadDirs(browse);})"
    ".catch(function(){toast('新建失败: 网络错误');});}\n"
    "fetch('/labels.json',{cache:'no-store'}).then(function(r){return r.json();})"
    ".then(function(j){if(!j||!j.labels||!j.labels.length)return;labels=j.labels;"
    "var bar=document.getElementById('lbar');bar.innerHTML='';nmEls=[];"
    "for(var i=0;i<labels.length;i++){var e=document.createElement('span');"
    "e.style.marginRight='12px';e.style.whiteSpace='nowrap';"
    "var sw=document.createElement('span');sw.className='sw';"
    "sw.style.background=(j.colors&&j.colors[i])||'#888';"
    "var nm=document.createElement('span');"
    "nm.textContent=labels[i];nmEls.push(nm);"
    "e.appendChild(sw);e.appendChild(nm);bar.appendChild(e);}"
    "bar.style.display='';}).catch(function(){});\n"
    "loadDirs('');\n"
    "</script></body></html>\n";

static void json_escape(const char *in, char *out, size_t n)
{
    size_t j = 0;
    for (const unsigned char *p = (const unsigned char *)in; *p && j + 7 < n; ++p) {
        unsigned char c = *p;
        if (c == '"' || c == '\\') {
            out[j++] = '\\';
            out[j++] = (char)c;
        } else if (c < 0x20) {
            j += (size_t)snprintf(out + j, n - j, "\\u%04x", c);
        } else {
            out[j++] = (char)c;
        }
    }
    out[j] = 0;
}

/* ------------------------------------------------------- 数据采集（data_get） */

/*
 * 网页“保存”把最新一帧原始 RGB（叠加前）以 JPEG q95 写入当前目录；
 * “保存至”在 DATA_ROOT 下浏览/切换目录，“新建”在浏览目录下建子目录。
 * 请求里的路径参数都是相对 DATA_ROOT 的路径；服务端做组件校验 + realpath
 * 约束，保证任何读写都落不到 DATA_ROOT 之外。
 */
#ifndef DATA_ROOT /* 主机自测可用 -DDATA_ROOT='"/tmp/..."' 覆盖 */
#define DATA_ROOT "/userdata/iris/data_get"
#endif
#define DATA_REL_MAX 200
#define DATA_NAME_MAX 120

typedef struct {
    pthread_mutex_t mu;
    char root[PATH_MAX];            /* DATA_ROOT 的 realpath */
    char cur_rel[DATA_REL_MAX + 1]; /* 当前保存目录（"" = root） */
    long saved;                     /* 本次运行保存张数 */
} DataStore;

static int seg_mkdir_p(const char *path); /* 定义在分割部分 */

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* 百分号解码（'+' 按字面量处理，URL 里的中文目录名也能用） */
static void url_decode(const char *in, char *out, size_t n)
{
    size_t j = 0;
    for (const char *p = in; *p && j + 1 < n; ++p) {
        if (p[0] == '%' && p[1] && p[2]) {
            int hi = hexval((unsigned char)p[1]);
            int lo = hexval((unsigned char)p[2]);
            if (hi >= 0 && lo >= 0) {
                out[j++] = (char)((hi << 4) | lo);
                p += 2;
                continue;
            }
        }
        out[j++] = *p;
    }
    out[j] = 0;
}

/* 相对路径是否安全：非绝对、无 . / .. 组件、无空组件、无控制字符/反斜杠 */
static int rel_is_safe(const char *rel)
{
    if (!rel[0]) return 1;
    if (rel[0] == '/' || strlen(rel) > DATA_REL_MAX) return 0;
    const char *p = rel;
    while (*p) {
        const char *s = p;
        while (*p && *p != '/') {
            unsigned char c = (unsigned char)*p;
            if (c < 0x20 || c == 0x7f || c == '\\') return 0;
            ++p;
        }
        const size_t len = (size_t)(p - s);
        if (len == 0) return 0;
        if ((len == 1 && s[0] == '.') ||
            (len == 2 && s[0] == '.' && s[1] == '.'))
            return 0;
        if (*p == '/') ++p;
    }
    return 1;
}

/* 请求参数 -> 解码、去掉尾部斜杠、安全检查 */
static int parse_rel(const char *raw, char *out, size_t n)
{
    char dec[DATA_REL_MAX * 3 + 8];
    if (!raw) return -1;
    url_decode(raw, dec, sizeof(dec));
    size_t l = strlen(dec);
    while (l > 0 && dec[l - 1] == '/') dec[--l] = 0;
    if (!rel_is_safe(dec) || strlen(dec) >= n) return -1;
    memcpy(out, dec, strlen(dec) + 1);
    return 0;
}

/* 相对路径 -> realpath 绝对路径，且必须仍在 DATA_ROOT 内且是目录 */
static int resolve_rel(DataStore *ds, const char *rel, char *abs, size_t n)
{
    char cand[PATH_MAX];
    char rp[PATH_MAX];
    int k = rel[0] ? snprintf(cand, sizeof(cand), "%s/%s", ds->root, rel)
                   : snprintf(cand, sizeof(cand), "%s", ds->root);
    if (k < 0 || (size_t)k >= sizeof(cand)) return -1;
    if (!realpath(cand, rp)) return -1;
    const size_t rl = strlen(ds->root);
    if (strncmp(rp, ds->root, rl) != 0) return -1;
    if (rp[rl] != '\0' && rp[rl] != '/') return -1;
    struct stat st;
    if (stat(rp, &st) != 0 || !S_ISDIR(st.st_mode)) return -1;
    if (strlen(rp) >= n) return -1;
    snprintf(abs, n, "%s", rp);
    return 0;
}

static int cmp_cstr(const void *a, const void *b)
{
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

/* 取 query 里的某个参数（原样，未解码）；成功返回 0 */
static int query_get(const char *q, const char *key, char *out, size_t n)
{
    const size_t kl = strlen(key);
    while (q && *q) {
        const char *amp = strchr(q, '&');
        const size_t len = amp ? (size_t)(amp - q) : strlen(q);
        const char *eq = (const char *)memchr(q, '=', len);
        if (eq && (size_t)(eq - q) == kl && strncmp(q, key, kl) == 0) {
            size_t vl = len - (size_t)(eq - q) - 1;
            if (vl >= n) vl = n - 1;
            memcpy(out, eq + 1, vl);
            out[vl] = 0;
            return 0;
        }
        q = amp ? amp + 1 : NULL;
    }
    return -1;
}

static void data_init(DataStore *ds)
{
    memset(ds, 0, sizeof(*ds));
    pthread_mutex_init(&ds->mu, NULL);
    if (seg_mkdir_p(DATA_ROOT) != 0)
        logmsg("数据采集: 无法创建 %s: %s", DATA_ROOT, strerror(errno));
    if (!realpath(DATA_ROOT, ds->root))
        snprintf(ds->root, sizeof(ds->root), "%s", DATA_ROOT);

    /* 记住上次选择的目录（存在 DATA_ROOT/.selected） */
    char sel[PATH_MAX + 16];
    snprintf(sel, sizeof(sel), "%s/.selected", ds->root);
    FILE *f = fopen(sel, "r");
    if (f) {
        char line[DATA_REL_MAX * 3 + 8] = {0};
        if (fgets(line, sizeof(line), f)) {
            line[strcspn(line, "\r\n")] = 0;
            char rel[DATA_REL_MAX + 1] = {0};
            char abs[PATH_MAX];
            if (parse_rel(line, rel, sizeof(rel)) == 0 && rel[0] &&
                resolve_rel(ds, rel, abs, sizeof(abs)) == 0)
                snprintf(ds->cur_rel, sizeof(ds->cur_rel), "%s", rel);
        }
        fclose(f);
    }
    if (ds->cur_rel[0])
        logmsg("数据采集目录: %s/%s（网页“保存”写入这里）", ds->root,
               ds->cur_rel);
    else
        logmsg("数据采集目录: %s（当前保存到根目录）", ds->root);
}

static int data_set_cur(DataStore *ds, const char *rel)
{
    char abs[PATH_MAX];
    if (resolve_rel(ds, rel, abs, sizeof(abs)) != 0) return -1;
    pthread_mutex_lock(&ds->mu);
    snprintf(ds->cur_rel, sizeof(ds->cur_rel), "%s", rel);
    char sel[PATH_MAX + 16];
    snprintf(sel, sizeof(sel), "%s/.selected", ds->root);
    FILE *f = fopen(sel, "w");
    if (f) {
        fprintf(f, "%s\n", rel);
        fclose(f);
    }
    pthread_mutex_unlock(&ds->mu);
    return 0;
}

static void json_send(int fd, int code, const char *reason, const char *body)
{
    send_http(fd, code, reason, "application/json; charset=utf-8", body,
              strlen(body));
}

static void json_error(int fd, int code, const char *reason, const char *msg)
{
    char esc[768];
    char body[832];
    json_escape(msg, esc, sizeof(esc));
    int n = snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}\n",
                     esc);
    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(body)) n = (int)sizeof(body) - 1;
    send_http(fd, code, reason, "application/json; charset=utf-8", body,
              (size_t)n);
}

/* 可增长字符串，用于拼目录列表 JSON */
static int sb_add(char **buf, size_t *len, size_t *cap, const char *s)
{
    const size_t n = strlen(s);
    if (*len + n + 1 > *cap) {
        size_t nc = *cap ? *cap * 2 : 1024;
        while (nc < *len + n + 1) nc *= 2;
        char *p = realloc(*buf, nc);
        if (!p) return -1;
        *buf = p;
        *cap = nc;
    }
    memcpy(*buf + *len, s, n + 1);
    *len += n;
    return 0;
}

/* GET /dirs.json?path=rel —— 列出 DATA_ROOT 下的子目录 */
static void data_send_dirs(int fd, DataStore *ds, const char *query)
{
    char raw[512] = {0};
    char rel[DATA_REL_MAX + 1] = {0};
    if (query_get(query, "path", raw, sizeof(raw)) == 0) {
        if (parse_rel(raw, rel, sizeof(rel)) != 0) {
            json_error(fd, 400, "Bad Request", "非法路径");
            return;
        }
    }
    char abs[PATH_MAX];
    if (resolve_rel(ds, rel, abs, sizeof(abs)) != 0) {
        json_error(fd, 404, "Not Found", "目录不存在");
        return;
    }

    char **names = NULL;
    int n = 0, cap = 0;
    DIR *d = opendir(abs);
    if (!d) {
        json_error(fd, 500, "Internal Server Error", "无法读取目录");
        return;
    }
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue; /* 跳过 . .. 和隐藏文件 */
        char p[PATH_MAX];
        int k = snprintf(p, sizeof(p), "%s/%s", abs, de->d_name);
        if (k < 0 || (size_t)k >= sizeof(p)) continue;
        struct stat st;
        if (stat(p, &st) != 0 || !S_ISDIR(st.st_mode)) continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 16;
            char **p2 = (char **)realloc(names, (size_t)cap * sizeof(*p2));
            if (!p2) break;
            names = p2;
        }
        names[n] = strdup(de->d_name);
        if (names[n]) n++;
    }
    closedir(d);
    if (n > 1) qsort(names, (size_t)n, sizeof(*names), cmp_cstr);

    char *out = NULL;
    size_t len = 0, ocap = 0;
    char tmp[PATH_MAX * 4];
    char esc[PATH_MAX * 2];
    json_escape(ds->root, esc, sizeof(esc));
    snprintf(tmp, sizeof(tmp), "{\"ok\":true,\"root\":\"%s\",\"path\":\"", esc);
    if (sb_add(&out, &len, &ocap, tmp) != 0) goto oom;
    json_escape(rel, esc, sizeof(esc));
    if (sb_add(&out, &len, &ocap, esc) != 0) goto oom;
    if (sb_add(&out, &len, &ocap, "\",\"cur\":\"") != 0) goto oom;
    pthread_mutex_lock(&ds->mu);
    json_escape(ds->cur_rel, esc, sizeof(esc));
    pthread_mutex_unlock(&ds->mu);
    if (sb_add(&out, &len, &ocap, esc) != 0) goto oom;
    if (sb_add(&out, &len, &ocap, "\",\"dirs\":[") != 0) goto oom;
    for (int i = 0; i < n; ++i) {
        json_escape(names[i], esc, sizeof(esc));
        snprintf(tmp, sizeof(tmp), "%s\"%s\"", i ? "," : "", esc);
        if (sb_add(&out, &len, &ocap, tmp) != 0) goto oom;
    }
    if (sb_add(&out, &len, &ocap, "]}\n") != 0) goto oom;
    json_send(fd, 200, "OK", out);
    free(out);
    for (int i = 0; i < n; ++i) free(names[i]);
    free(names);
    return;

oom:
    json_error(fd, 500, "Internal Server Error", "内存不足");
    free(out);
    for (int i = 0; i < n; ++i) free(names[i]);
    free(names);
}

/* GET /setdir?path=rel —— 切换当前保存目录 */
static void data_setdir(int fd, DataStore *ds, const char *query)
{
    char raw[512] = {0};
    char rel[DATA_REL_MAX + 1] = {0};
    if (query_get(query, "path", raw, sizeof(raw)) != 0 ||
        parse_rel(raw, rel, sizeof(rel)) != 0) {
        json_error(fd, 400, "Bad Request", "非法路径");
        return;
    }
    if (data_set_cur(ds, rel) != 0) {
        json_error(fd, 404, "Not Found", "目录不存在");
        return;
    }
    char esc[PATH_MAX * 2];
    char body[PATH_MAX * 2 + 64];
    json_escape(rel, esc, sizeof(esc));
    snprintf(body, sizeof(body), "{\"ok\":true,\"cur\":\"%s\"}\n", esc);
    json_send(fd, 200, "OK", body);
}

static int name_is_safe(const char *s)
{
    const size_t l = strlen(s);
    if (l == 0 || l > DATA_NAME_MAX) return 0;
    if (!strcmp(s, ".") || !strcmp(s, "..")) return 0;
    for (const unsigned char *p = (const unsigned char *)s; *p; ++p)
        if (*p < 0x20 || *p == 0x7f || *p == '/' || *p == '\\') return 0;
    return 1;
}

/* GET /mkdir?parent=rel&name=xxx —— 在目录下新建子目录 */
static void data_mkdir(int fd, DataStore *ds, const char *query)
{
    char praw[512] = {0}, nraw[512] = {0};
    char parent[DATA_REL_MAX + 1] = {0}, name[DATA_NAME_MAX + 4] = {0};
    if (query_get(query, "parent", praw, sizeof(praw)) != 0)
        praw[0] = 0;
    if (query_get(query, "name", nraw, sizeof(nraw)) != 0 ||
        parse_rel(praw, parent, sizeof(parent)) != 0) {
        json_error(fd, 400, "Bad Request", "参数错误");
        return;
    }
    url_decode(nraw, name, sizeof(name));
    if (!name_is_safe(name)) {
        json_error(fd, 400, "Bad Request", "文件夹名称不合法");
        return;
    }

    char abs[PATH_MAX];
    if (resolve_rel(ds, parent, abs, sizeof(abs)) != 0) {
        json_error(fd, 404, "Not Found", "父目录不存在");
        return;
    }
    char path[PATH_MAX];
    int k = snprintf(path, sizeof(path), "%s/%s", abs, name);
    if (k < 0 || (size_t)k >= sizeof(path)) {
        json_error(fd, 400, "Bad Request", "路径过长");
        return;
    }
    if (mkdir(path, 0755) != 0) {
        json_error(fd, errno == EEXIST ? 409 : 500,
                   errno == EEXIST ? "Conflict" : "Internal Server Error",
                   errno == EEXIST ? "文件夹已存在" : strerror(errno));
        return;
    }

    char rel[DATA_REL_MAX * 2 + 4];
    if (parent[0])
        snprintf(rel, sizeof(rel), "%s/%s", parent, name);
    else
        snprintf(rel, sizeof(rel), "%s", name);
    char resc[PATH_MAX * 2], nesc[PATH_MAX * 2];
    char body[PATH_MAX * 6];
    json_escape(rel, resc, sizeof(resc));
    json_escape(name, nesc, sizeof(nesc));
    snprintf(body, sizeof(body), "{\"ok\":true,\"rel\":\"%s\",\"name\":\"%s\"}\n",
             resc, nesc);
    json_send(fd, 200, "OK", body);
}

static void stamp_compact(char *buf, size_t n)
{
    struct timespec ts;
    struct tm tm;
    char base[32];
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm);
    strftime(base, sizeof(base), "%Y%m%d_%H%M%S", &tm);
    snprintf(buf, n, "%s_%03ld", base, ts.tv_nsec / 1000000L);
}

/* GET /save —— 把最新一帧原始 RGB 以 JPEG q95 存到当前目录 */
static void data_save(int fd, FrameHub *hub, DataStore *ds)
{
    unsigned char *raw = NULL, *jpg = NULL;
    int w = 0, h = 0;
    size_t jlen = 0;

    pthread_mutex_lock(&hub->mu);
    if (hub->raw && hub->raw_len && hub->raw_w > 0 && hub->raw_h > 0) {
        raw = (unsigned char *)malloc(hub->raw_len);
        if (raw) {
            memcpy(raw, hub->raw, hub->raw_len);
            w = hub->raw_w;
            h = hub->raw_h;
        }
    }
    if (!raw && hub->jpeg_len > 0) {
        jpg = (unsigned char *)malloc(hub->jpeg_len);
        if (jpg) {
            memcpy(jpg, hub->jpeg, hub->jpeg_len);
            jlen = hub->jpeg_len;
        }
    }
    pthread_mutex_unlock(&hub->mu);

    if (!raw && !jpg) {
        json_error(fd, 503, "Service Unavailable", "还没有可用画面");
        return;
    }

    Buffer enc = {0};
    int ok = 0;
    if (raw) {
        ok = encode_jpeg(raw, w, h, 95, &enc) == 0 && enc.data && enc.len > 0;
        free(raw);
    } else {
        enc.data = jpg;
        enc.len = jlen;
        ok = 1;
    }
    if (!ok) {
        free(enc.data);
        json_error(fd, 500, "Internal Server Error", "JPEG 编码失败");
        return;
    }

    char rel[DATA_REL_MAX + 1] = {0};
    char dir[PATH_MAX];
    pthread_mutex_lock(&ds->mu);
    snprintf(rel, sizeof(rel), "%s", ds->cur_rel);
    pthread_mutex_unlock(&ds->mu);
    if (resolve_rel(ds, rel, dir, sizeof(dir)) != 0) {
        /* 当前目录被删/改名：退回 data_get 根目录，别让“保存”失效 */
        if (resolve_rel(ds, "", dir, sizeof(dir)) != 0) {
            free(enc.data);
            json_error(fd, 500, "Internal Server Error", "保存目录不可用");
            return;
        }
        if (rel[0]) {
            logmsg("数据采集: 目录 data_get/%s 已不存在，改存根目录", rel);
            rel[0] = 0;
            pthread_mutex_lock(&ds->mu);
            ds->cur_rel[0] = 0;
            pthread_mutex_unlock(&ds->mu);
            char sel[PATH_MAX + 16];
            snprintf(sel, sizeof(sel), "%s/.selected", ds->root);
            FILE *sf = fopen(sel, "w");
            if (sf) {
                fprintf(sf, "\n");
                fclose(sf);
            }
        }
    }

    char stamp[64], name[128], path[PATH_MAX];
    stamp_compact(stamp, sizeof(stamp));
    for (int i = 1;; ++i) {
        if (i == 1)
            snprintf(name, sizeof(name), "img_%s.jpg", stamp);
        else
            snprintf(name, sizeof(name), "img_%s_%d.jpg", stamp, i);
        int k = snprintf(path, sizeof(path), "%s/%s", dir, name);
        if (k < 0 || (size_t)k >= sizeof(path)) {
            free(enc.data);
            json_error(fd, 500, "Internal Server Error", "保存路径过长");
            return;
        }
        if (access(path, F_OK) != 0 || i >= 100) break;
    }

    int wok = 0;
    FILE *f = fopen(path, "wb");
    if (f) {
        wok = fwrite(enc.data, 1, enc.len, f) == enc.len;
        if (fclose(f) != 0) wok = 0;
    }
    const size_t bytes = enc.len;
    free(enc.data);
    if (!wok) {
        json_error(fd, 500, "Internal Server Error", "写文件失败");
        return;
    }

    pthread_mutex_lock(&ds->mu);
    ds->saved++;
    long saved_n = ds->saved;
    pthread_mutex_unlock(&ds->mu);

    char resc[PATH_MAX * 2], dresc[PATH_MAX * 2];
    char body[PATH_MAX * 6];
    json_escape(name, resc, sizeof(resc));
    json_escape(rel, dresc, sizeof(dresc));
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"file\":\"%s\",\"dir\":\"%s\",\"bytes\":%zu,"
             "\"saved_total\":%ld}\n",
             resc, dresc, bytes, saved_n);
    json_send(fd, 200, "OK", body);
}

static void send_status(int fd, FrameHub *hub)
{
    char src[2 * sizeof(hub->source) + 1];
    char ratios[SEG_MAX_CLASSES * 12 + 2];
    char regs[SEG_MAX_REGIONS * 72 + 4];
    char body[4096];
    const int infer_on = __atomic_load_n(&hub->infer_on, __ATOMIC_RELAXED);
    pthread_mutex_lock(&hub->mu);
    json_escape(hub->source, src, sizeof(src));
    double fps = hub->fps;
    /* 超过 2s 没有新帧就把帧率显示成 0，而不是停在旧值 */
    if (hub->last_pub_ms <= 0 || now_ms() - hub->last_pub_ms > 2000.0)
        fps = 0.0;
    int rl = 0;
    ratios[rl++] = '[';
    for (int k = 0; k < hub->n_class && k < SEG_MAX_CLASSES; ++k) {
        int w = snprintf(ratios + rl, sizeof(ratios) - (size_t)rl,
                         "%s%.2f", k ? "," : "", hub->cls_ratio[k]);
        if (w < 0 || (size_t)(rl + w) >= sizeof(ratios))
            break;
        rl += w;
    }
    ratios[rl++] = ']';
    ratios[rl] = 0;
    /* 脏污连通域：坐标是原始帧像素（x/y 左上角，w/h 宽高），area 是模型
     * 网格上的像素数，按面积从大到小 */
    int gl = 0;
    regs[gl++] = '[';
    for (int i = 0; i < hub->n_region && i < SEG_MAX_REGIONS; ++i) {
        const SegRegion *r = &hub->region[i];
        int w = snprintf(regs + gl, sizeof(regs) - (size_t)gl,
                         "%s{\"cls\":%d,\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d,"
                         "\"area\":%ld}",
                         i ? "," : "", r->cls, r->x, r->y, r->w, r->h, r->area);
        if (w < 0 || (size_t)(gl + w) >= sizeof(regs))
            break;
        gl += w;
    }
    regs[gl++] = ']';
    regs[gl] = 0;
    const int pub_width = __atomic_load_n(&g_pub_width, __ATOMIC_RELAXED);
    int n = snprintf(body, sizeof(body),
                     "{\"seq\":%lu,\"width\":%d,\"height\":%d,\"pub_width\":%d,"
                     "\"fps\":%.2f,"
                     "\"frames\":%lu,\"jpeg_bytes\":%zu,\"source\":\"%s\","
                     "\"infer\":%s,\"infer_on\":%s,\"has_model\":%s,"
                     "\"dirty\":%s,\"n_class\":%d,\"cls_ratio\":%s,"
                     "\"target_class\":%d,\"target_ratio\":%.2f,"
                     "\"fg_ratio\":%.2f,\"infer_ms\":%.1f,"
                     "\"n_region\":%d,\"regions\":%s}\n",
                     hub->seq, hub->width, hub->height, pub_width, fps,
                     hub->frames, hub->jpeg_len, src,
                     hub->infer ? "true" : "false",
                     infer_on ? "true" : "false",
                     hub->has_model ? "true" : "false",
                     hub->dirty ? "true" : "false", hub->n_class, ratios,
                     hub->target_class, hub->target_ratio, hub->fg_ratio,
                     hub->infer_ms, hub->n_region, regs);
    pthread_mutex_unlock(&hub->mu);
    if (n < 0)
        n = 0;
    if ((size_t)n >= sizeof(body))
        n = (int)sizeof(body) - 1;
    send_http(fd, 200, "OK", "application/json; charset=utf-8", body, (size_t)n);
}

/* GET /labels.json —— 类别名 + 叠加颜色（网页图例用） */
static void send_labels_json(int fd)
{
    if (g_n_class <= 0) {
        json_error(fd, 503, "Service Unavailable", "未加载模型");
        return;
    }
    char names[SEG_MAX_CLASSES * (2 * SEG_CLASS_NAME_MAX + 4)];
    char colors[SEG_MAX_CLASSES * 12];
    int nl = 0, cl = 0;
    names[nl++] = '[';
    colors[cl++] = '[';
    for (int k = 0; k < g_n_class && k < SEG_MAX_CLASSES; ++k) {
        char esc[2 * SEG_CLASS_NAME_MAX + 1];
        json_escape(g_cls_name[k], esc, sizeof(esc));
        int w = snprintf(names + nl, sizeof(names) - (size_t)nl, "%s\"%s\"",
                         k ? "," : "", esc);
        if (w < 0 || (size_t)(nl + w) >= sizeof(names))
            break;
        nl += w;
        const uint8_t *c = SEG_PALETTE_RGB[k];
        w = snprintf(colors + cl, sizeof(colors) - (size_t)cl,
                     "%s\"#%02x%02x%02x\"", k ? "," : "", c[0], c[1], c[2]);
        if (w < 0 || (size_t)(cl + w) >= sizeof(colors))
            break;
        cl += w;
    }
    names[nl++] = ']';
    names[nl] = 0;
    colors[cl++] = ']';
    colors[cl] = 0;

    char body[sizeof(names) + sizeof(colors) + 64];
    int n = snprintf(body, sizeof(body), "{\"labels\":%s,\"colors\":%s}\n",
                     names, colors);
    if (n < 0)
        n = 0;
    if ((size_t)n >= sizeof(body))
        n = (int)sizeof(body) - 1;
    send_http(fd, 200, "OK", "application/json; charset=utf-8", body,
              (size_t)n);
}

/* GET /infer?on=0|1 —— 网页开关：实时推理叠加 或 仅推流 */
static void send_infer_toggle(int fd, FrameHub *hub, const char *query)
{
    if (!hub->has_model) {
        json_error(fd, 503, "Service Unavailable", "启动时未加载模型");
        return;
    }
    char raw[16] = {0};
    if (query_get(query, "on", raw, sizeof(raw)) != 0) {
        json_error(fd, 400, "Bad Request", "缺少 on 参数");
        return;
    }
    int on;
    if (!strcmp(raw, "1") || !strcasecmp(raw, "true") ||
        !strcasecmp(raw, "on"))
        on = 1;
    else if (!strcmp(raw, "0") || !strcasecmp(raw, "false") ||
             !strcasecmp(raw, "off"))
        on = 0;
    else {
        json_error(fd, 400, "Bad Request", "on 只能是 0/1");
        return;
    }
    __atomic_store_n(&hub->infer_on, on, __ATOMIC_RELAXED);
    logmsg("脏污检测: %s（网页开关）", on ? "开启" : "关闭，仅推流");
    char body[96];
    snprintf(body, sizeof(body), "{\"ok\":true,\"on\":%s,\"available\":true}\n",
             on ? "true" : "false");
    json_send(fd, 200, "OK", body);
}

/* GET /res?w=N —— 网页切换推流分辨率（0 = 原始尺寸，否则等比缩到 N 宽） */
static void send_res_toggle(int fd, const char *query)
{
    char raw[16] = {0};
    if (query_get(query, "w", raw, sizeof(raw)) != 0) {
        json_error(fd, 400, "Bad Request", "缺少 w 参数");
        return;
    }
    char *end = NULL;
    const long v = strtol(raw, &end, 10);
    if (!raw[0] || (end && *end) || v < 0 || v > 8192) {
        json_error(fd, 400, "Bad Request", "w 必须是 0..8192 的整数");
        return;
    }
    const int w = pub_width_set((int)v);
    if (w > 0)
        logmsg("推流宽度: %d（网页切换，等比缩放）", w);
    else
        logmsg("推流宽度: 原始尺寸（网页切换，不缩放）");
    char body[64];
    snprintf(body, sizeof(body), "{\"ok\":true,\"w\":%d}\n", w);
    json_send(fd, 200, "OK", body);
}

static void send_snapshot(int fd, FrameHub *hub)
{
    unsigned char *copy = NULL;
    size_t len = 0;
    pthread_mutex_lock(&hub->mu);
    if (hub->jpeg_len > 0) {
        copy = malloc(hub->jpeg_len);
        if (copy) {
            memcpy(copy, hub->jpeg, hub->jpeg_len);
            len = hub->jpeg_len;
        }
    }
    pthread_mutex_unlock(&hub->mu);
    if (!copy) {
        static const char msg[] = "no frame yet\n";
        send_http(fd, 503, "Service Unavailable", "text/plain; charset=utf-8",
                  msg, sizeof(msg) - 1);
        return;
    }
    send_http(fd, 200, "OK", "image/jpeg", copy, len);
    free(copy);
}

/* MJPEG：每个客户端一个线程，按 seq 等待新帧 */
static void mjpeg_client(int fd, FrameHub *hub)
{
    static const char hdr[] =
        "HTTP/1.0 200 OK\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "\r\n";
    if (send_all(fd, hdr, sizeof(hdr) - 1) != 0)
        return;

    unsigned char *buf = NULL;
    size_t cap = 0;
    unsigned long last = 0;
    while (!hub->quit && !g_stop) {
        pthread_mutex_lock(&hub->mu);
        while (!hub->quit && !g_stop && hub->seq == last)
            pthread_cond_wait(&hub->cv, &hub->mu);
        if (hub->quit || g_stop) {
            pthread_mutex_unlock(&hub->mu);
            break;
        }
        last = hub->seq;
        if (hub->jpeg_len > cap) {
            unsigned char *p = realloc(buf, hub->jpeg_len);
            if (!p) {
                pthread_mutex_unlock(&hub->mu);
                break;
            }
            buf = p;
            cap = hub->jpeg_len;
        }
        size_t len = hub->jpeg_len;
        memcpy(buf, hub->jpeg, len);
        pthread_mutex_unlock(&hub->mu);

        char part[128];
        int n = snprintf(part, sizeof(part),
                         "--frame\r\nContent-Type: image/jpeg\r\n"
                         "Content-Length: %zu\r\n\r\n",
                         len);
        if (n < 0 || send_all(fd, part, (size_t)n) != 0 ||
            send_all(fd, buf, len) != 0 ||
            send_all(fd, "\r\n", 2) != 0)
            break;
    }
    free(buf);
}

typedef struct {
    int port;
    FrameHub *hub;
    DataStore *data; /* 数据采集（可空：nullptr 时相关接口返回 503） */
    int listen_fd;
    pthread_t tid;
    volatile int stop;
    pthread_mutex_t cmu;
    int clients;
    int max_clients;
} HttpServer;

typedef struct {
    int fd;
    HttpServer *srv;
} ClientArg;

static void http_handle(int fd, HttpServer *srv)
{
    char req[4096];
    size_t got = 0;
    req[0] = 0;
    while (got + 1 < sizeof(req)) {
        ssize_t n = recv(fd, req + got, sizeof(req) - 1 - got, 0);
        if (n <= 0)
            return;
        got += (size_t)n;
        req[got] = 0;
        if (strchr(req, '\n')) /* 请求行已完整，足够了 */
            break;
    }

    char method[16] = {0};
    char path[1024] = {0};
    if (sscanf(req, "%15s %1023s", method, path) != 2) {
        static const char msg[] = "bad request\n";
        send_http(fd, 400, "Bad Request", "text/plain; charset=utf-8",
                  msg, sizeof(msg) - 1);
        return;
    }
    if (strcmp(method, "GET") != 0 && strcmp(method, "HEAD") != 0) {
        static const char msg[] = "method not allowed\n";
        send_http(fd, 405, "Method Not Allowed", "text/plain; charset=utf-8",
                  msg, sizeof(msg) - 1);
        return;
    }
    char *q = strchr(path, '?');
    if (q)
        *q = 0;

    if (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0) {
        send_http(fd, 200, "OK", "text/html; charset=utf-8",
                  PAGE_HTML, sizeof(PAGE_HTML) - 1);
    } else if (strcmp(path, "/stream.mjpg") == 0) {
        mjpeg_client(fd, srv->hub); /* 长连接，函数返回后由调用者关闭 */
    } else if (strcmp(path, "/status.json") == 0) {
        send_status(fd, srv->hub);
    } else if (strcmp(path, "/snapshot.jpg") == 0) {
        send_snapshot(fd, srv->hub);
    } else if (strcmp(path, "/infer") == 0) {
        send_infer_toggle(fd, srv->hub, q ? q + 1 : "");
    } else if (strcmp(path, "/res") == 0) {
        send_res_toggle(fd, q ? q + 1 : "");
    } else if (strcmp(path, "/labels.json") == 0) {
        send_labels_json(fd);
    } else if (strcmp(path, "/save") == 0) {
        if (srv->data)
            data_save(fd, srv->hub, srv->data);
        else
            json_error(fd, 503, "Service Unavailable", "数据采集未启用");
    } else if (strcmp(path, "/dirs.json") == 0) {
        if (srv->data)
            data_send_dirs(fd, srv->data, q ? q + 1 : "");
        else
            json_error(fd, 503, "Service Unavailable", "数据采集未启用");
    } else if (strcmp(path, "/setdir") == 0) {
        if (srv->data)
            data_setdir(fd, srv->data, q ? q + 1 : "");
        else
            json_error(fd, 503, "Service Unavailable", "数据采集未启用");
    } else if (strcmp(path, "/mkdir") == 0) {
        if (srv->data)
            data_mkdir(fd, srv->data, q ? q + 1 : "");
        else
            json_error(fd, 503, "Service Unavailable", "数据采集未启用");
    } else {
        static const char msg[] = "not found\n";
        send_http(fd, 404, "Not Found", "text/plain; charset=utf-8",
                  msg, sizeof(msg) - 1);
    }
}

static void *client_thread(void *arg)
{
    ClientArg *ca = (ClientArg *)arg;
    HttpServer *s = ca->srv;
    http_handle(ca->fd, s);
    close(ca->fd);
    pthread_mutex_lock(&s->cmu);
    s->clients--;
    pthread_mutex_unlock(&s->cmu);
    free(ca);
    return NULL;
}

static void *http_thread(void *arg)
{
    HttpServer *s = (HttpServer *)arg;
    struct pollfd pfd;
    pfd.fd = s->listen_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    while (!s->stop) {
        int rc = poll(&pfd, 1, 200);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (rc == 0)
            continue;

        struct sockaddr_in cli;
        socklen_t cl = sizeof(cli);
        int fd = accept(s->listen_fd, (struct sockaddr *)&cli, &cl);
        if (fd < 0) {
            if (errno == EINTR)
                continue;
            if (s->stop)
                break;
            usleep(10000);
            continue;
        }

        pthread_mutex_lock(&s->cmu);
        int over = s->clients >= s->max_clients;
        if (!over)
            s->clients++;
        pthread_mutex_unlock(&s->cmu);
        if (over) {
            static const char msg[] = "too many clients\n";
            send_http(fd, 503, "Service Unavailable", "text/plain; charset=utf-8",
                      msg, sizeof(msg) - 1);
            close(fd);
            continue;
        }

        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        ClientArg *ca = malloc(sizeof(*ca));
        if (!ca) {
            close(fd);
            pthread_mutex_lock(&s->cmu);
            s->clients--;
            pthread_mutex_unlock(&s->cmu);
            continue;
        }
        ca->fd = fd;
        ca->srv = s;
        pthread_t t;
        if (pthread_create(&t, NULL, client_thread, ca) != 0) {
            close(fd);
            free(ca);
            pthread_mutex_lock(&s->cmu);
            s->clients--;
            pthread_mutex_unlock(&s->cmu);
            continue;
        }
        pthread_detach(t);
    }
    return NULL;
}

static int http_start(HttpServer *s, int port, FrameHub *hub, DataStore *data)
{
    memset(s, 0, sizeof(*s));
    s->port = port;
    s->hub = hub;
    s->data = data;
    s->listen_fd = -1;
    pthread_mutex_init(&s->cmu, NULL);
    s->max_clients = MAX_CLIENTS;

    int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        logmsg("socket 失败: %s", strerror(errno));
        return -1;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)s->port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        logmsg("bind :%d 失败: %s", s->port, strerror(errno));
        close(fd);
        return -1;
    }
    if (listen(fd, 16) != 0) {
        logmsg("listen 失败: %s", strerror(errno));
        close(fd);
        return -1;
    }
    s->listen_fd = fd;
    if (pthread_create(&s->tid, NULL, http_thread, s) != 0) {
        logmsg("HTTP 线程创建失败");
        close(fd);
        s->listen_fd = -1;
        return -1;
    }
    return 0;
}

/* -------------------------------------------------------- 帧源 1：目录监听 */

static int newest_png(const char *dir, char *out, size_t n)
{
    DIR *d = opendir(dir);
    if (!d)
        return -1;
    struct dirent *e;
    struct stat st, best;
    memset(&best, 0, sizeof(best));
    int found = 0;
    char best_name[NAME_MAX + 1] = {0};
    while ((e = readdir(d)) != NULL) {
        if (!has_png_ext(e->d_name))
            continue;
        char path[PATH_MAX];
        if (snprintf(path, sizeof(path), "%s/%s", dir, e->d_name) >= (int)sizeof(path))
            continue;
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode))
            continue;
        if (!found ||
            st.st_mtim.tv_sec > best.st_mtim.tv_sec ||
            (st.st_mtim.tv_sec == best.st_mtim.tv_sec &&
             st.st_mtim.tv_nsec > best.st_mtim.tv_nsec)) {
            best = st;
            snprintf(best_name, sizeof(best_name), "%s", e->d_name);
            found = 1;
        }
    }
    closedir(d);
    if (!found)
        return -1;
    snprintf(out, n, "%s/%s", dir, best_name);
    return 0;
}

/* -------------------------------------------- 模型分割（镜头脏污检测） */

/*
 * 与 infer_board.c 一致：4 类语义分割，class 0 = 背景，class 1/2/3 = 三种
 * 脏污类（默认目标类 class 3）。每帧解码出的 RGB 在内存里做：
 *   长边缩放到 640 → 5:3 居中裁切 → 384x640 → packed NV12（BT.601 video）
 *   → hbDNN → 逐像素类别 → 三类合计占比。
 * 合计占比超过 --dirty-ratio 就按 test.py / infer_board 的 overlay 方式：三种
 * 脏污类分别与各自调色板颜色 50% 混合画回当前帧，再叠一行 "LENS DIRTY"。
 * 全程不落盘。
 */

typedef struct {
    void *lib;

    int32_t (*hbSysAllocCachedMem)(hbSysMem *, uint32_t);
    int32_t (*hbSysFlushMem)(hbSysMem *, int32_t);
    int32_t (*hbSysFreeMem)(hbSysMem *);
    int32_t (*hbDNNInitializeFromFiles)(hbPackedDNNHandle_t *, char const **, int32_t);
    int32_t (*hbDNNRelease)(hbPackedDNNHandle_t);
    int32_t (*hbDNNGetModelNameList)(char const ***, int32_t *, hbPackedDNNHandle_t);
    int32_t (*hbDNNGetModelHandle)(hbDNNHandle_t *, hbPackedDNNHandle_t, char const *);
    int32_t (*hbDNNGetInputCount)(int32_t *, hbDNNHandle_t);
    int32_t (*hbDNNGetInputTensorProperties)(hbDNNTensorProperties *, hbDNNHandle_t, int32_t);
    int32_t (*hbDNNGetOutputCount)(int32_t *, hbDNNHandle_t);
    int32_t (*hbDNNGetOutputTensorProperties)(hbDNNTensorProperties *, hbDNNHandle_t, int32_t);
    int32_t (*hbDNNInfer)(hbDNNTaskHandle_t *, hbDNNTensor **, hbDNNTensor const *,
                          hbDNNHandle_t, hbDNNInferCtrlParam *);
    int32_t (*hbDNNWaitTaskDone)(hbDNNTaskHandle_t, int32_t);
    int32_t (*hbDNNReleaseTask)(hbDNNTaskHandle_t);
    char const *(*hbDNNGetVersion)(void);

    hbPackedDNNHandle_t packed;
    hbDNNHandle_t handle;
    hbDNNTensorProperties in_props;
    int input_count;
    int output_count;
    hbDNNTensorProperties *out_props;
    hbDNNTensor input;
    hbDNNTensor *outputs;
    int dtype_s8;

    int mw, mh;              /* 模型输入宽高 */
    int n_class;             /* labels.txt 类别数（0 号 = clean） */
    char cls_name[SEG_MAX_CLASSES][SEG_CLASS_NAME_MAX];
    int target_class;
    double dirty_ratio;      /* 占比阈值（0..100） */
    char save_dir[PATH_MAX]; /* 脏污对比图目录，空串 = 不保存 */
    int save_max;            /* 目录内最多保留张数，<=0 = 不保存 */
    int save_interval_ms;    /* 两张对比图的最小间隔（ms），0 = 每脏污帧都存 */
    double last_save_ms;     /* 上次保存对比图的时刻 */
    long saved_n;            /* 本次运行累计保存张数 */
    double last_save_log;
    char rec_dir[PATH_MAX];  /* -a/-b 录制目录，空串 = 不录制 */
    int rec_max;             /* 最多保留帧数（滚动删最旧），<=0 = 不保存 */
    int rec_dirty_only;      /* 1 = 只录脏污帧（-b），0 = 每帧都录（-a） */
    int rec_on;              /* 1 = 正在录制（空间不足后清零） */
    long rec_n;              /* 本次运行累计录制帧数 */
    double rec_mb;           /* 粗略累计写入量（MB） */
    double last_rec_log;
    uint8_t *nv12;           /* mw*mh*3/2 临时输入 */
    uint8_t *labels;         /* mw*mh 类别图 */
    int region_min_px;       /* 连通域最小面积（模型像素），更小的当噪点 */
    uint8_t *cc_vis;         /* mw*mh，连通域 BFS 访问标记 */
    int *cc_stk;             /* mw*mh，BFS 栈（像素线性下标） */
} SegEngine;

#define SEG_LOAD_SYM(e, name)                                                    \
    do {                                                                         \
        *(void **)(&(e)->name) = dlsym((e)->lib, #name);                         \
        if (!(e)->name) {                                                        \
            logmsg("libdnn.so 缺少符号 %s", #name);                              \
            missing++;                                                           \
        }                                                                        \
    } while (0)

static int seg_input_hw(const hbDNNTensorProperties *p, int *h, int *w)
{
    const int nd = p->validShape.numDimensions;
    const int *d = p->validShape.dimensionSize;
    if (nd == 4) {
        if (d[3] == 3) {
            *h = d[1];
            *w = d[2];
        } else if (d[1] == 3) {
            *h = d[2];
            *w = d[3];
        } else if (p->tensorLayout == HB_DNN_LAYOUT_NCHW) {
            *h = d[2];
            *w = d[3];
        } else {
            *h = d[1];
            *w = d[2];
        }
    } else if (nd == 3) {
        if (d[2] == 3) {
            *h = d[0];
            *w = d[1];
        } else if (d[0] == 3) {
            *h = d[1];
            *w = d[2];
        } else if (p->tensorLayout == HB_DNN_LAYOUT_NCHW) {
            *h = d[1];
            *w = d[2];
        } else {
            *h = d[0];
            *w = d[1];
        }
    } else {
        return -1;
    }
    return (*h > 0 && *w > 0) ? 0 : -1;
}

static void seg_input_aligned_hw(const hbDNNTensorProperties *p, int vh, int vw,
                                 int *ah, int *aw)
{
    hbDNNTensorProperties q = *p;
    q.validShape = p->alignedShape;
    int h = 0, w = 0;
    if (seg_input_hw(&q, &h, &w) == 0 && h >= vh && w >= vw &&
        (size_t)h * w * 3 / 2 <= (size_t)p->alignedByteSize) {
        *ah = h;
        *aw = w;
    } else {
        *ah = vh;
        *aw = vw;
    }
}

static int seg_fill_input(SegEngine *e, const uint8_t *nv12, int w, int h)
{
    size_t need = (size_t)w * h * 3 / 2;
    uint32_t cap = e->input.sysMem[0].memSize ? e->input.sysMem[0].memSize
                                              : (uint32_t)e->in_props.alignedByteSize;
    if (need > cap) {
        logmsg("NV12 需要 %zu 字节，模型输入只有 %u 字节", need, cap);
        return -1;
    }
    uint8_t *dst = (uint8_t *)e->input.sysMem[0].virAddr;

    int ah = h, aw = w;
    seg_input_aligned_hw(&e->in_props, h, w, &ah, &aw);
    if (aw == w && ah == h) {
        memcpy(dst, nv12, need);
    } else {
        for (int y = 0; y < h; ++y)
            memcpy(dst + (size_t)y * aw, nv12 + (size_t)y * w, (size_t)w);
        uint8_t *uvd = dst + (size_t)ah * aw;
        const uint8_t *uvs = nv12 + (size_t)w * h;
        for (int y = 0; y < h / 2; ++y)
            memcpy(uvd + (size_t)y * aw, uvs + (size_t)y * w, (size_t)w);
    }
    if (e->hbSysFlushMem(&e->input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN) != 0) {
        logmsg("输入 flush 失败");
        return -1;
    }
    return 0;
}

static int seg_decode_labels(SegEngine *e, int W, int H, uint8_t *labels)
{
    const hbDNNTensorProperties *p = &e->out_props[0];
    const hbDNNTensor *t = &e->outputs[0];
    const int nd = p->validShape.numDimensions;
    const int *d = p->validShape.dimensionSize;
    int oh = 0, ow = 0, oc = 1;
    int channels_first = 0;
    if (nd == 4) {
        if (d[1] == H && d[2] == W) {
            oh = d[1];
            ow = d[2];
            oc = d[3];
        } else if (d[2] == H && d[3] == W) {
            oh = d[2];
            ow = d[3];
            oc = d[1];
            channels_first = 1;
        }
    } else if (nd == 3) {
        if (d[0] == H && d[1] == W) {
            oh = d[0];
            ow = d[1];
            oc = d[2];
        } else if (d[2] <= 4) {
            oh = d[0];
            ow = d[1];
            oc = d[2];
        } else {
            oh = d[1];
            ow = d[2];
            oc = d[0];
            channels_first = 1;
        }
    } else if (nd == 2) {
        oh = d[0];
        ow = d[1];
    } else {
        logmsg("无法解析输出 shape（numDimensions=%d）", nd);
        return -1;
    }
    if (oh != H || ow != W) {
        logmsg("输出尺寸 %dx%d 与模型输入 %dx%d 不一致", ow, oh, W, H);
        return -1;
    }

    long elems = (long)oh * ow * oc;
    long capacity = p->alignedByteSize;
    if (capacity < elems) {
        logmsg("输出缓冲区过小 (%ld < %ld)", capacity, elems);
        return -1;
    }
    const uint8_t *base = (const uint8_t *)t->sysMem[0].virAddr;
    int contiguous = (capacity == elems);

    long stride_h = 0, stride_w = 0, stride_c = 0;
    if (!contiguous) {
        if (nd == 4) {
            stride_h = p->stride[1] > 0 ? p->stride[1] : (long)ow * oc;
            stride_w = p->stride[2] > 0 ? p->stride[2] : oc;
            stride_c = p->stride[3] > 0 ? p->stride[3] : 1;
        } else if (nd == 3) {
            stride_h = p->stride[0] > 0 ? p->stride[0] : (long)ow * oc;
            stride_w = p->stride[1] > 0 ? p->stride[1] : oc;
            stride_c = p->stride[2] > 0 ? p->stride[2] : 1;
        } else {
            stride_h = p->stride[0] > 0 ? p->stride[0] : ow;
            stride_w = p->stride[1] > 0 ? p->stride[1] : 1;
        }
        long max_off = (long)(oh - 1) * stride_h + (long)(ow - 1) * stride_w +
                       (long)(oc - 1) * stride_c;
        if (max_off + 1 > capacity) {
            contiguous = 1;
            stride_h = stride_w = 0;
            stride_c = 1;
        }
    }

    long ch_stride = channels_first ? (long)oh * ow : 1;
    if (!contiguous) ch_stride = stride_c;

#define SEG_FETCH(idx) (e->dtype_s8 ? (int)((const int8_t *)base)[idx] : (int)base[idx])

    for (int y = 0; y < oh; ++y) {
        for (int x = 0; x < ow; ++x) {
            long off;
            if (contiguous) {
                off = ((long)y * ow + x) * (channels_first ? 1 : oc);
            } else {
                off = (long)y * stride_h + (long)x * stride_w;
            }
            int label;
            if (oc == 1) {
                label = SEG_FETCH(off);
            } else {
                int best = 0;
                float best_v = -1e30f;
                for (int c = 0; c < oc; ++c) {
                    int raw = SEG_FETCH(off + (long)c * ch_stride);
                    float v = (float)raw;
                    if (p->quantiType == SCALE && p->scale.scaleLen == oc) {
                        float zp = (p->scale.zeroPointLen == oc)
                                       ? (float)p->scale.zeroPointData[c]
                                       : 0.0f;
                        v = ((float)raw - zp) * p->scale.scaleData[c];
                    }
                    if (v > best_v) {
                        best_v = v;
                        best = c;
                    }
                }
                label = best;
            }
            if (label < 0 || label >= e->n_class) {
                logmsg("输出类别 %d 超出 labels 的 %d 类", label, e->n_class);
                return -1;
            }
            labels[(long)y * ow + x] = (uint8_t)label;
        }
    }
#undef SEG_FETCH
    return 0;
}

static int seg_infer(SegEngine *e, const uint8_t *nv12, int w, int h,
                     uint8_t *labels)
{
    if (seg_fill_input(e, nv12, w, h) != 0)
        return -1;

    hbDNNInferCtrlParam ctrl;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl);
    hbDNNTaskHandle_t task = NULL;
    hbDNNTensor *outs = e->outputs;
    int32_t ret = e->hbDNNInfer(&task, &outs, &e->input, e->handle, &ctrl);
    if (ret != 0) {
        logmsg("hbDNNInfer 失败 (%d)", ret);
        return -1;
    }
    e->outputs = outs;
    ret = e->hbDNNWaitTaskDone(task, 0);
    if (ret != 0) {
        logmsg("hbDNNWaitTaskDone 失败 (%d)", ret);
        e->hbDNNReleaseTask(task);
        return -1;
    }
    for (int i = 0; i < e->output_count; ++i) {
        if (e->outputs[i].sysMem[0].virAddr)
            e->hbSysFlushMem(&e->outputs[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    }
    int rc = seg_decode_labels(e, w, h, labels);
    e->hbDNNReleaseTask(task);
    return rc;
}

static int seg_mkdir_p(const char *path);

/* labels.txt：每行一个类名，第一行 = clean。找不到时用内置 class0..class15。 */
static void seg_load_labels(SegEngine *e, const char *labels_path,
                            const char *model_path)
{
    char cand[PATH_MAX];
    const char *lp = labels_path;
    if (!lp) {
        /* 默认找模型同目录下的 labels.txt */
        const char *slash = strrchr(model_path, '/');
        if (slash) {
            int k = snprintf(cand, sizeof(cand), "%.*s/labels.txt",
                             (int)(slash - model_path), model_path);
            if (k > 0 && (size_t)k < sizeof(cand) && access(cand, R_OK) == 0)
                lp = cand;
        } else if (access("labels.txt", R_OK) == 0) {
            lp = "labels.txt";
        }
    } else if (access(lp, R_OK) != 0) {
        logmsg("labels 文件不存在，改用内置类名: %s", lp);
        lp = NULL;
    }

    e->n_class = 0;
    if (lp) {
        FILE *f = fopen(lp, "r");
        if (f) {
            char line[256];
            while (e->n_class < SEG_MAX_CLASSES && fgets(line, sizeof(line), f)) {
                line[strcspn(line, "\r\n")] = 0;
                char *p = line;
                while (*p == ' ' || *p == '\t')
                    ++p;
                char *end = p + strlen(p);
                while (end > p && (end[-1] == ' ' || end[-1] == '\t'))
                    *--end = 0;
                if (!*p || *p == '#')
                    continue;
                snprintf(e->cls_name[e->n_class], SEG_CLASS_NAME_MAX, "%.*s",
                         SEG_CLASS_NAME_MAX - 1, p);
                e->n_class++;
            }
            fclose(f);
        }
        if (e->n_class > 0)
            logmsg("labels: %s（%d 类）", lp, e->n_class);
    }
    if (e->n_class <= 0) {
        e->n_class = SEG_MAX_CLASSES;
        logmsg("labels: 内置 class0..class%d", SEG_MAX_CLASSES - 1);
    }
    for (int i = e->n_class; i < SEG_MAX_CLASSES; ++i)
        snprintf(e->cls_name[i], SEG_CLASS_NAME_MAX, "class%d", i);

    /* 供 /labels.json 与网页图例使用（启动后只读） */
    g_n_class = e->n_class;
    memcpy(g_cls_name, e->cls_name, sizeof(g_cls_name));
}

static int seg_open(SegEngine *e, const char *model_path,
                    const char *labels_path, int target_class,
                    double dirty_ratio, int region_min_px,
                    const char *save_dir, int save_max,
                    int save_interval_ms, const char *rec_dir, int rec_max,
                    int rec_dirty_only)
{
    memset(e, 0, sizeof(*e));
    e->target_class = target_class;
    e->dirty_ratio = dirty_ratio;
    e->region_min_px = region_min_px < 0 ? 0 : region_min_px;
    e->save_max = save_max;
    e->save_interval_ms = save_interval_ms;
    if (save_dir && save_dir[0] && save_max > 0) {
        if (snprintf(e->save_dir, sizeof(e->save_dir), "%s", save_dir) >=
            (int)sizeof(e->save_dir)) {
            logmsg("--save-dir 路径过长");
            return -1;
        }
        if (seg_mkdir_p(e->save_dir) != 0) {
            logmsg("无法创建脏污对比图目录: %s", e->save_dir);
            return -1;
        }
    }
    if (rec_dir && rec_dir[0]) {
        if (snprintf(e->rec_dir, sizeof(e->rec_dir), "%s", rec_dir) >=
            (int)sizeof(e->rec_dir)) {
            logmsg("--record-dir 路径过长");
            return -1;
        }
        if (seg_mkdir_p(e->rec_dir) != 0) {
            logmsg("无法创建录制目录: %s", e->rec_dir);
            return -1;
        }
        e->rec_max = rec_max;
        e->rec_dirty_only = rec_dirty_only;
        e->rec_on = rec_max > 0;
    }

    seg_load_labels(e, labels_path, model_path);
    if (target_class < 0 || target_class >= e->n_class) {
        logmsg("--target-class 必须在 0..%d 之间（模型共 %d 类）",
               e->n_class - 1, e->n_class);
        return -1;
    }

    const char *cands[] = {
        getenv("INFER_BOARD_LIBDNN"), "libdnn.so", "libdnn.so.1",
        "/usr/lib/libdnn.so", "/usr/lib/libdnn.so.1"};
    char dlerr[256] = "";
    dlerror();
    for (size_t i = 0; i < sizeof(cands) / sizeof(cands[0]); ++i) {
        if (!cands[i] || !cands[i][0])
            continue;
        e->lib = dlopen(cands[i], RTLD_NOW | RTLD_GLOBAL);
        if (e->lib)
            break;
        const char *msg = dlerror();
        if (msg && msg[0] && dlerr[0] == '\0')
            snprintf(dlerr, sizeof(dlerr), "%s", msg);
    }
    if (!e->lib) {
        logmsg("无法加载 libdnn.so: %s", dlerr[0] ? dlerr : "unknown");
        return -1;
    }

    int missing = 0;
    SEG_LOAD_SYM(e, hbSysAllocCachedMem);
    SEG_LOAD_SYM(e, hbSysFlushMem);
    SEG_LOAD_SYM(e, hbSysFreeMem);
    SEG_LOAD_SYM(e, hbDNNInitializeFromFiles);
    SEG_LOAD_SYM(e, hbDNNRelease);
    SEG_LOAD_SYM(e, hbDNNGetModelNameList);
    SEG_LOAD_SYM(e, hbDNNGetModelHandle);
    SEG_LOAD_SYM(e, hbDNNGetInputCount);
    SEG_LOAD_SYM(e, hbDNNGetInputTensorProperties);
    SEG_LOAD_SYM(e, hbDNNGetOutputCount);
    SEG_LOAD_SYM(e, hbDNNGetOutputTensorProperties);
    SEG_LOAD_SYM(e, hbDNNInfer);
    SEG_LOAD_SYM(e, hbDNNWaitTaskDone);
    SEG_LOAD_SYM(e, hbDNNReleaseTask);
    SEG_LOAD_SYM(e, hbDNNGetVersion);
    if (missing)
        return -1;

    char const *model_file = model_path;
    int32_t ret = e->hbDNNInitializeFromFiles(&e->packed, &model_file, 1);
    if (ret != 0) {
        logmsg("加载模型失败 (%d): %s", ret, model_path);
        return -1;
    }
    char const **names = NULL;
    int32_t name_count = 0;
    if (e->hbDNNGetModelNameList(&names, &name_count, e->packed) != 0 ||
        name_count < 1) {
        logmsg("获取模型名失败");
        return -1;
    }
    if (e->hbDNNGetModelHandle(&e->handle, e->packed, names[0]) != 0) {
        logmsg("获取模型句柄失败");
        return -1;
    }
    e->hbDNNGetInputCount(&e->input_count, e->handle);
    e->hbDNNGetOutputCount(&e->output_count, e->handle);
    if (e->input_count != 1) {
        logmsg("模型有 %d 个输入，仅支持单输入 NV12 分割模型", e->input_count);
        return -1;
    }
    if (e->hbDNNGetInputTensorProperties(&e->in_props, e->handle, 0) != 0) {
        logmsg("获取输入属性失败");
        return -1;
    }
    if (seg_input_hw(&e->in_props, &e->mh, &e->mw) != 0) {
        logmsg("无法解析模型输入尺寸");
        return -1;
    }

    e->out_props = (hbDNNTensorProperties *)calloc((size_t)e->output_count,
                                                   sizeof(hbDNNTensorProperties));
    e->outputs = (hbDNNTensor *)calloc((size_t)e->output_count, sizeof(hbDNNTensor));
    e->nv12 = (uint8_t *)malloc((size_t)e->mw * e->mh * 3 / 2);
    e->labels = (uint8_t *)malloc((size_t)e->mw * e->mh);
    e->cc_vis = (uint8_t *)malloc((size_t)e->mw * e->mh);
    e->cc_stk = (int *)malloc((size_t)e->mw * e->mh * sizeof(int));
    if (!e->out_props || !e->outputs || !e->nv12 || !e->labels ||
        !e->cc_vis || !e->cc_stk) {
        logmsg("内存不足");
        return -1;
    }
    for (int i = 0; i < e->output_count; ++i) {
        if (e->hbDNNGetOutputTensorProperties(&e->out_props[i], e->handle, i) != 0) {
            logmsg("获取输出 %d 属性失败", i);
            return -1;
        }
        e->outputs[i].properties = e->out_props[i];
        if (e->out_props[i].alignedByteSize > 0 &&
            e->hbSysAllocCachedMem(&e->outputs[i].sysMem[0],
                                   (uint32_t)e->out_props[i].alignedByteSize) != 0) {
            logmsg("输出内存分配失败");
            return -1;
        }
    }
    e->input.properties = e->in_props;
    uint32_t in_bytes = (uint32_t)e->in_props.alignedByteSize;
    if (in_bytes == 0 ||
        e->hbSysAllocCachedMem(&e->input.sysMem[0], in_bytes) != 0) {
        logmsg("输入内存分配失败");
        return -1;
    }
    memset(e->input.sysMem[0].virAddr, 0, in_bytes);
    int t = e->out_props[0].tensorType;
    if (t != HB_DNN_TENSOR_TYPE_S8 && t != HB_DNN_TENSOR_TYPE_U8) {
        logmsg("输出 dtype=%d，期望 uint8/int8 类别图", t);
        return -1;
    }
    e->dtype_s8 = (t == HB_DNN_TENSOR_TYPE_S8);

    logmsg("模型: %s (%s), 输入 %dx%d NV12, %d 类, 目标类 %d(%s), "
           "脏污合计阈值 %.2f%%, 连通域最小 %d px",
           names[0], model_path, e->mw, e->mh, e->n_class, e->target_class,
           e->cls_name[e->target_class], e->dirty_ratio, e->region_min_px);
    if (e->save_dir[0] && e->save_interval_ms > 0)
        logmsg("脏污对比图: %s（最多 %d 张，超出删最旧，最小间隔 %d ms）",
               e->save_dir, e->save_max, e->save_interval_ms);
    else if (e->save_dir[0])
        logmsg("脏污对比图: %s（最多 %d 张，超出删最旧，每脏污帧都存）",
               e->save_dir, e->save_max);
    else
        logmsg("脏污对比图: 不保存");
    if (e->rec_on) {
        logmsg("逐帧录制(%s): %s（orig_*.jpg + det_*.jpg，只保留最新 %d 帧，"
               "超出删最旧）",
               e->rec_dirty_only ? "-b 仅脏污帧" : "-a 每帧", e->rec_dir,
               e->rec_max);
    } else if (e->rec_dir[0]) {
        logmsg("逐帧录制: 不保存（帧数上限为 0）");
    }
    return 0;
}

static void seg_close(SegEngine *e)
{
    for (int i = 0; i < e->output_count; ++i) {
        if (e->outputs && e->outputs[i].sysMem[0].virAddr)
            e->hbSysFreeMem(&e->outputs[i].sysMem[0]);
    }
    if (e->input.sysMem[0].virAddr && e->hbSysFreeMem)
        e->hbSysFreeMem(&e->input.sysMem[0]);
    if (e->packed && e->hbDNNRelease)
        e->hbDNNRelease(e->packed);
    free(e->outputs);
    free(e->out_props);
    free(e->nv12);
    free(e->labels);
    free(e->cc_vis);
    free(e->cc_stk);
    if (e->lib)
        dlclose(e->lib);
    memset(e, 0, sizeof(*e));
}

/* --- 预处理：与 infer_board.c 的 load_and_preprocess + rgb_to_nv12 一致 --- */

static uint8_t *seg_resize_bilinear(const uint8_t *src, int sw, int sh, int dw, int dh)
{
    uint8_t *dst = (uint8_t *)malloc((size_t)dw * dh * 3);
    if (!dst)
        return NULL;
    const double sx = (double)sw / dw;
    const double sy = (double)sh / dh;
    for (int y = 0; y < dh; ++y) {
        double fy = (y + 0.5) * sy - 0.5;
        if (fy < 0) fy = 0;
        int y0 = (int)fy;
        if (y0 > sh - 1) y0 = sh - 1;
        int y1 = y0 + 1 < sh ? y0 + 1 : y0;
        double wy = fy - y0;
        for (int x = 0; x < dw; ++x) {
            double fx = (x + 0.5) * sx - 0.5;
            if (fx < 0) fx = 0;
            int x0 = (int)fx;
            if (x0 > sw - 1) x0 = sw - 1;
            int x1 = x0 + 1 < sw ? x0 + 1 : x0;
            double wx = fx - x0;
            const uint8_t *p00 = src + ((size_t)y0 * sw + x0) * 3;
            const uint8_t *p01 = src + ((size_t)y0 * sw + x1) * 3;
            const uint8_t *p10 = src + ((size_t)y1 * sw + x0) * 3;
            const uint8_t *p11 = src + ((size_t)y1 * sw + x1) * 3;
            uint8_t *d = dst + ((size_t)y * dw + x) * 3;
            for (int c = 0; c < 3; ++c) {
                double v = (1 - wy) * ((1 - wx) * p00[c] + wx * p01[c]) +
                           wy * ((1 - wx) * p10[c] + wx * p11[c]);
                int iv = (int)(v + 0.5);
                d[c] = (uint8_t)(iv < 0 ? 0 : iv > 255 ? 255 : iv);
            }
        }
    }
    return dst;
}

static void seg_rgb_to_nv12(const uint8_t *rgb, int w, int h, uint8_t *nv12)
{
    uint8_t *yp = nv12;
    uint8_t *uv = nv12 + (size_t)w * h;
    for (int y = 0; y < h; ++y) {
        const uint8_t *row = rgb + (size_t)y * w * 3;
        for (int x = 0; x < w; ++x) {
            int r = row[x * 3 + 0], g = row[x * 3 + 1], b = row[x * 3 + 2];
            int yv = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
            yp[(size_t)y * w + x] = (uint8_t)(yv < 16 ? 16 : yv > 235 ? 235 : yv);
        }
    }
    /* 与 cv2.COLOR_BGR2YUV_I420 一致：色度取 2x2 块左上角像素 */
    for (int y = 0; y < h / 2; ++y) {
        const uint8_t *row = rgb + (size_t)(2 * y) * w * 3;
        for (int x = 0; x < w / 2; ++x) {
            int r = row[(2 * x) * 3 + 0];
            int g = row[(2 * x) * 3 + 1];
            int b = row[(2 * x) * 3 + 2];
            int u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
            int v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
            size_t i = (size_t)y * w + x * 2;
            uv[i] = (uint8_t)(u < 0 ? 0 : u > 255 ? 255 : u);
            uv[i + 1] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
        }
    }
}

/* 原图 → 模型输入的裁切几何（overlay 时把类别图映射回原图用） */
typedef struct {
    double scale;   /* 原图 → 缩放后坐标 */
    int nw, nh;     /* 缩放后尺寸 */
    int x0, y0;     /* 居中裁切原点（缩放后坐标） */
    int cw, ch;     /* 裁切尺寸 */
    int mw, mh;     /* 模型输入尺寸 */
} SegGeom;

static uint8_t *seg_preprocess(const uint8_t *src, int w, int h, int mw, int mh,
                               SegGeom *g)
{
    const int long_edge = mw > mh ? mw : mh;
    const double aspect = (double)mw / mh;
    const double scale = (double)long_edge / (h > w ? h : w);
    int nw = (int)(w * scale + 0.5);
    int nh = (int)(h * scale + 0.5);
    if (nw < 1) nw = 1;
    if (nh < 1) nh = 1;

    uint8_t *resized;
    if (nw == w && nh == h) {
        resized = NULL; /* 不缩放，直接从原图裁 */
    } else {
        resized = seg_resize_bilinear(src, w, h, nw, nh);
        if (!resized)
            return NULL;
        src = resized;
    }

    int cw, ch;
    if ((double)nw / nh > aspect) {
        cw = (int)(nh * aspect + 0.5);
        ch = nh;
    } else {
        cw = nw;
        ch = (int)(nw / aspect + 0.5);
    }
    if (cw < 1) cw = 1;
    if (ch < 1) ch = 1;
    if (cw > nw) cw = nw;
    if (ch > nh) ch = nh;
    const int x0 = (nw - cw) / 2;
    const int y0 = (nh - ch) / 2;

    uint8_t *crop = (uint8_t *)malloc((size_t)cw * ch * 3);
    if (!crop) {
        free(resized);
        return NULL;
    }
    for (int y = 0; y < ch; ++y)
        memcpy(crop + (size_t)y * cw * 3,
               src + ((size_t)(y0 + y) * nw + x0) * 3, (size_t)cw * 3);
    free(resized);

    uint8_t *final_img = crop;
    if (cw != mw || ch != mh) {
        final_img = seg_resize_bilinear(crop, cw, ch, mw, mh);
        free(crop);
        if (!final_img)
            return NULL;
    }

    g->scale = scale;
    g->nw = nw;
    g->nh = nh;
    g->x0 = x0;
    g->y0 = y0;
    g->cw = cw;
    g->ch = ch;
    g->mw = mw;
    g->mh = mh;
    return final_img;
}

/* --- overlay：与 infer_board.c 的 overlay 一致（50% 调色板混合） --- */

static void seg_fill_rect(uint8_t *rgb, int W, int H, int x0, int y0, int x1,
                          int y1, const uint8_t c[3])
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > W) x1 = W;
    if (y1 > H) y1 = H;
    for (int y = y0; y < y1; ++y) {
        uint8_t *row = rgb + ((size_t)y * W + x0) * 3;
        for (int x = x0; x < x1; ++x, row += 3) {
            row[0] = c[0];
            row[1] = c[1];
            row[2] = c[2];
        }
    }
}

static void seg_draw_text(uint8_t *rgb, int W, int H, int x0, int y0,
                          const char *text, int scale)
{
    static char vbuf[16384];
    int nq = stb_easy_font_print(0, 0, (char *)text, NULL, vbuf, (int)sizeof(vbuf));
    const float *v = (const float *)vbuf;
    const uint8_t white[3] = {255, 255, 255};
    for (int q = 0; q < nq; ++q) {
        const float *vq = v + (size_t)q * 16;
        float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
        for (int k = 0; k < 4; ++k) {
            float x = vq[k * 4 + 0];
            float y = vq[k * 4 + 1];
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;
        }
        seg_fill_rect(rgb, W, H, x0 + (int)(minx * scale),
                      y0 + (int)(miny * scale), x0 + (int)(maxx * scale) + 1,
                      y0 + (int)(maxy * scale) + 1, white);
    }
}

static void seg_draw_banner(uint8_t *rgb, int W, int H, const char *text)
{
    const int scale = 2, pad = 6;
    int tw = stb_easy_font_width((char *)text) * scale;
    int th = stb_easy_font_height((char *)text) * scale;
    const uint8_t bg[3] = {0, 0, 0};
    seg_fill_rect(rgb, W, H, 8, 8, 8 + tw + pad * 2, 8 + th + pad * 2, bg);
    seg_draw_text(rgb, W, H, 8 + pad, 8 + pad, text, scale);
}

/* --- 脏污连通域：8 邻域 BFS，统计面积和包围盒 --------------------------- */

/* 按面积从大到小插进固定长度数组（超出 SEG_MAX_REGIONS 的丢掉） */
static void seg_cc_insert(SegRegion *arr, int *n, const SegRegion *r)
{
    int pos = *n;
    while (pos > 0 && arr[pos - 1].area < r->area)
        --pos;
    if (pos >= SEG_MAX_REGIONS)
        return;
    if (*n < SEG_MAX_REGIONS)
        ++*n;
    memmove(&arr[pos + 1], &arr[pos],
            (size_t)(*n - 1 - pos) * sizeof(*arr));
    arr[pos] = *r;
}

/* 在模型网格上找脏污连通域（class>=1 且同一类才算连在一起），把包围盒
 * 映射回原图坐标后写进 info->region（按面积从大到小，最多 SEG_MAX_REGIONS 个）。
 * 小面积（< region_min_px 模型像素）的当噪声丢弃。 */
static void seg_find_regions(SegEngine *e, const SegGeom *g, InferInfo *info)
{
    info->n_region = 0;
    const int mw = e->mw, mh = e->mh;
    if (!e->cc_vis || !e->cc_stk || mw <= 0 || mh <= 0 || e->region_min_px < 0 ||
        g->scale <= 0.0 || g->cw <= 0 || g->ch <= 0)
        return;

    memset(e->cc_vis, 0, (size_t)mw * mh);
    SegRegion best[SEG_MAX_REGIONS];
    int nbest = 0;
    static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int sy = 0; sy < mh; ++sy) {
        for (int sx = 0; sx < mw; ++sx) {
            const long start = (long)sy * mw + sx;
            const uint8_t cls = e->labels[start];
            if (cls == 0 || cls >= e->n_class || e->cc_vis[start])
                continue;

            int sp = 0;
            e->cc_stk[sp++] = (int)start;
            e->cc_vis[start] = 1;
            long area = 0;
            int x0 = sx, x1 = sx, y0 = sy, y1 = sy;
            while (sp > 0) {
                const int idx = e->cc_stk[--sp];
                const int x = idx % mw, y = idx / mw;
                ++area;
                if (x < x0) x0 = x;
                if (x > x1) x1 = x;
                if (y < y0) y0 = y;
                if (y > y1) y1 = y;
                for (int k = 0; k < 8; ++k) {
                    const int nx = x + dx[k], ny = y + dy[k];
                    if (nx < 0 || nx >= mw || ny < 0 || ny >= mh)
                        continue;
                    const long ni = (long)ny * mw + nx;
                    if (e->cc_vis[ni] || e->labels[ni] != cls)
                        continue;
                    e->cc_vis[ni] = 1;
                    e->cc_stk[sp++] = (int)ni;
                }
            }
            if (area < e->region_min_px)
                continue;

            /* 模型像素 -> 原图像素：反演 seg_overlay 的映射
             * x = (x0 + mx*cw/mw) / scale */
            const double sx0 = (g->x0 + (double)x0 * g->cw / mw) / g->scale;
            const double sx1 =
                (g->x0 + (double)(x1 + 1) * g->cw / mw) / g->scale;
            const double sy0 = (g->y0 + (double)y0 * g->ch / mh) / g->scale;
            const double sy1 =
                (g->y0 + (double)(y1 + 1) * g->ch / mh) / g->scale;
            int wx0 = (int)(sx0 + 0.5), wx1 = (int)(sx1 + 0.5);
            int wy0 = (int)(sy0 + 0.5), wy1 = (int)(sy1 + 0.5);
            if (wx1 <= wx0) wx1 = wx0 + 1;
            if (wy1 <= wy0) wy1 = wy0 + 1;

            SegRegion r;
            memset(&r, 0, sizeof(r));
            r.cls = cls;
            r.area = area;
            r.x = wx0;
            r.y = wy0;
            r.w = wx1 - wx0;
            r.h = wy1 - wy0;
            seg_cc_insert(best, &nbest, &r);
        }
    }
    info->n_region = nbest;
    if (nbest > 0)
        memcpy(info->region, best, (size_t)nbest * sizeof(best[0]));
}

/* 画矩形边框（向内 t 像素），坐标越界自动裁剪 */
static void seg_rect_border(uint8_t *rgb, int W, int H, int x0, int y0, int x1,
                            int y1, const uint8_t c[3], int t)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > W) x1 = W;
    if (y1 > H) y1 = H;
    if (x1 <= x0 || y1 <= y0)
        return;
    if (t > (x1 - x0)) t = x1 - x0;
    if (t > (y1 - y0)) t = y1 - y0;
    if (t < 1)
        return;
    seg_fill_rect(rgb, W, H, x0, y0, x1, y0 + t, c);
    seg_fill_rect(rgb, W, H, x0, y1 - t, x1, y1, c);
    seg_fill_rect(rgb, W, H, x0, y0, x0 + t, y1, c);
    seg_fill_rect(rgb, W, H, x1 - t, y0, x1, y1, c);
}

/* 把连通域包围盒画到画面上：外层 1px 黑、内层 2px 类别色，深色/浅色背景
 * 都看得清。只画 info->region 里的前 n_region 个。 */
static void seg_draw_regions(uint8_t *rgb, int W, int H, const InferInfo *info)
{
    static const uint8_t black[3] = {0, 0, 0};
    for (int i = 0; i < info->n_region && i < SEG_MAX_REGIONS; ++i) {
        const SegRegion *r = &info->region[i];
        if (r->cls <= 0 || r->cls >= SEG_MAX_CLASSES || r->w <= 0 || r->h <= 0)
            continue;
        int x0 = r->x, y0 = r->y, x1 = r->x + r->w, y1 = r->y + r->h;
        if (x1 <= 0 || y1 <= 0 || x0 >= W || y0 >= H)
            continue;
        seg_rect_border(rgb, W, H, x0 - 1, y0 - 1, x1 + 1, y1 + 1, black, 1);
        seg_rect_border(rgb, W, H, x0, y0, x1, y1, SEG_PALETTE_RGB[r->cls], 2);
    }
}

/* 与 test.py / infer_board.c 的 save_visualization 一致：
 * 所有前景类（label > 0）按各自调色板颜色做 50% 混合，背景保持原样。
 * mask_out 非空时顺带画出纯色掩码图（三联图的中间那张）。 */
static void seg_overlay(const uint8_t *labels, const SegGeom *g, uint8_t *rgb,
                        int W, int H, int n_class, const InferInfo *info,
                        uint8_t *mask_out)
{
    if (g->scale <= 0.0 || g->cw <= 0 || g->ch <= 0)
        return;

    int ox0 = (int)(g->x0 / g->scale + 0.5);
    int ox1 = (int)((g->x0 + g->cw) / g->scale + 0.5);
    int oy0 = (int)(g->y0 / g->scale + 0.5);
    int oy1 = (int)((g->y0 + g->ch) / g->scale + 0.5);
    if (ox0 < 0) ox0 = 0;
    if (oy0 < 0) oy0 = 0;
    if (ox1 > W) ox1 = W;
    if (oy1 > H) oy1 = H;

    const int mw = g->mw, mh = g->mh;
    for (int y = oy0; y < oy1; ++y) {
        int my = (int)((y / g->scale - g->y0) * mh / g->ch);
        if (my < 0) my = 0;
        if (my >= mh) my = mh - 1;
        const uint8_t *labrow = labels + (size_t)my * mw;
        uint8_t *row = rgb + (size_t)y * W * 3;
        uint8_t *mrow = mask_out ? mask_out + (size_t)y * W * 3 : NULL;
        for (int x = ox0; x < ox1; ++x) {
            int mx = (int)((x / g->scale - g->x0) * mw / g->cw);
            if (mx < 0) mx = 0;
            if (mx >= mw) mx = mw - 1;
            int idx = labrow[mx];
            if (idx < 0 || idx >= n_class)
                idx = 0;
            const uint8_t *color = SEG_PALETTE_RGB[idx];
            if (mrow)
                memcpy(mrow + (size_t)x * 3, color, 3);
            if (idx == 0) /* clean 类不叠加（labels > 0 才是脏污） */
                continue;
            uint8_t *p = row + (size_t)x * 3;
            for (int c = 0; c < 3; ++c)
                p[c] = (uint8_t)((p[c] + color[c]) >> 1);
        }
    }

    seg_draw_regions(rgb, W, H, info);

    char banner[64];
    snprintf(banner, sizeof(banner), "LENS DIRTY  %.1f%%  N=%d", info->fg_ratio,
             info->n_region);
    seg_draw_banner(rgb, W, H, banner);
}

/* --- 脏污对比图落盘：input | mask | overlay 三联图，横向拼接成一张 JPEG --- */

static int seg_mkdir_p(const char *path)
{
    char tmp[PATH_MAX];
    if (snprintf(tmp, sizeof(tmp), "%s", path) >= (int)sizeof(tmp))
        return -1;
    for (char *p = tmp + 1; *p; ++p) {
        if (*p != '/')
            continue;
        *p = '\0';
        if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
            return -1;
        *p = '/';
    }
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
        return -1;
    return 0;
}

/* 只统计/删除 dirty_*.jpg；数量 >= max 时删掉 mtime 最旧的一张 */
static void seg_trim_dir(const char *dir, int max)
{
    for (;;) {
        DIR *d = opendir(dir);
        if (!d)
            return;
        struct dirent *de;
        struct stat st, oldest;
        int count = 0, have_old = 0;
        char oldpath[PATH_MAX] = {0};
        memset(&oldest, 0, sizeof(oldest));
        while ((de = readdir(d)) != NULL) {
            if (strncmp(de->d_name, "dirty_", 6) != 0)
                continue;
            const char *dot = strrchr(de->d_name, '.');
            if (!dot || strcasecmp(dot + 1, "jpg") != 0)
                continue;
            char p[PATH_MAX];
            if (snprintf(p, sizeof(p), "%s/%s", dir, de->d_name) >=
                (int)sizeof(p))
                continue;
            if (stat(p, &st) != 0)
                continue;
            count++;
            if (!have_old ||
                st.st_mtim.tv_sec < oldest.st_mtim.tv_sec ||
                (st.st_mtim.tv_sec == oldest.st_mtim.tv_sec &&
                 st.st_mtim.tv_nsec < oldest.st_mtim.tv_nsec)) {
                oldest = st;
                snprintf(oldpath, sizeof(oldpath), "%s", p);
                have_old = 1;
            }
        }
        closedir(d);
        if (!have_old || count < max)
            return;
        if (unlink(oldpath) != 0)
            return;
    }
}

/* 从 /dir/<帧名>.png 取出 <帧名>，用作录制文件名 */
static void path_stem(const char *path, char *out, size_t sz)
{
    const char *b = strrchr(path, '/');
    b = b ? b + 1 : path;
    snprintf(out, sz, "%s", b);
    char *dot = strrchr(out, '.');
    if (dot && dot != out)
        *dot = '\0';
}

/* 写一个 JPEG（先写 .tmp 再 rename，避免出现半张图） */
static int seg_write_jpg(const char *dir, const char *prefix, const char *stem,
                         const void *data, size_t len)
{
    char path[PATH_MAX], tmp[PATH_MAX];
    if (snprintf(path, sizeof(path), "%s/%s_%s.jpg", dir, prefix, stem) >=
            (int)sizeof(path) ||
        snprintf(tmp, sizeof(tmp), "%s.tmp", path) >= (int)sizeof(tmp))
        return -1;
    FILE *f = fopen(tmp, "wb");
    if (!f)
        return -1;
    int ok = fwrite(data, 1, len, f) == len;
    if (fclose(f) != 0)
        ok = 0;
    if (!ok || rename(tmp, path) != 0) {
        unlink(tmp);
        return -1;
    }
    return 0;
}

/*
 * 录制目录滚动上限：按 orig_*.jpg 统计帧数，超过 max 就删掉 mtime 最旧的
 * 一整对（orig_ + det_）。只处理本程序生成的文件名。
 */
static void seg_trim_record_dir(const char *dir, int max)
{
    if (max <= 0)
        return;
    for (;;) {
        DIR *d = opendir(dir);
        if (!d)
            return;
        struct dirent *de;
        struct stat st, oldest;
        int count = 0, have_old = 0;
        char oldpath[PATH_MAX] = {0};
        memset(&oldest, 0, sizeof(oldest));
        while ((de = readdir(d)) != NULL) {
            if (strncmp(de->d_name, "orig_", 5) != 0 ||
                !str_ends_with_ci(de->d_name, ".jpg"))
                continue;
            char p[PATH_MAX];
            if (snprintf(p, sizeof(p), "%s/%s", dir, de->d_name) >=
                (int)sizeof(p))
                continue;
            if (stat(p, &st) != 0)
                continue;
            count++;
            if (!have_old ||
                st.st_mtim.tv_sec < oldest.st_mtim.tv_sec ||
                (st.st_mtim.tv_sec == oldest.st_mtim.tv_sec &&
                 st.st_mtim.tv_nsec < oldest.st_mtim.tv_nsec)) {
                oldest = st;
                snprintf(oldpath, sizeof(oldpath), "%s", p);
                have_old = 1;
            }
        }
        closedir(d);
        if (!have_old || count < max)
            return;
        const char *bn = strrchr(oldpath, '/');
        bn = bn ? bn + 1 : oldpath;
        char det[PATH_MAX];
        int ok = unlink(oldpath) == 0;
        if (ok &&
            snprintf(det, sizeof(det), "%s/det_%s", dir, bn + 5) <
                (int)sizeof(det))
            unlink(det);
        if (!ok)
            return;
    }
}

/* 录制前检查：磁盘剩余不足时关闭录制 */
static int seg_record_ready(SegEngine *e)
{
    if (!e->rec_on)
        return 0;
    struct statvfs vfs;
    if (statvfs(e->rec_dir, &vfs) == 0) {
        const double mb =
            (double)vfs.f_bavail * (double)vfs.f_frsize / (1024.0 * 1024.0);
        if (mb < RECORD_MIN_FREE_MB) {
            e->rec_on = 0;
            logmsg("剩余空间仅 %.0f MB，停止逐帧录制（已录 %ld 帧）", mb,
                   e->rec_n);
            return 0;
        }
    }
    return 1;
}

/* 把同一帧的原图/检测图各存一张 JPG */
static void seg_record_frame(SegEngine *e, const char *stem,
                             const Buffer *orig, const Buffer *vis)
{
    if (!e->rec_on || !orig->data || !vis->data)
        return;
    seg_trim_record_dir(e->rec_dir, e->rec_max);
    const int ok_o =
        seg_write_jpg(e->rec_dir, "orig", stem, orig->data, orig->len) == 0;
    const int ok_v =
        seg_write_jpg(e->rec_dir, "det", stem, vis->data, vis->len) == 0;
    const double t = now_ms();
    if (!ok_o || !ok_v) {
        if (t - e->last_rec_log >= 5000.0) {
            e->last_rec_log = t;
            logmsg("逐帧录制写盘失败（orig %s, det %s），检查 %s 空间/权限",
                   ok_o ? "ok" : "fail", ok_v ? "ok" : "fail", e->rec_dir);
        }
        return;
    }
    e->rec_n++;
    e->rec_mb += (double)(orig->len + vis->len) / (1024.0 * 1024.0);
    if (t - e->last_rec_log >= 5000.0) {
        e->last_rec_log = t;
        logmsg("已录制 %ld 帧（保留最新 %d 帧，约 %.1f MB）-> %s", e->rec_n,
               e->rec_max, e->rec_mb, e->rec_dir);
    }
}

/* 现在是否到了可以存对比图的时刻（限速；0 间隔 = 每脏污帧都存） */
static int seg_save_due(const SegEngine *e)
{
    if (!e->save_dir[0] || e->save_max <= 0)
        return 0;
    if (e->save_interval_ms <= 0 || e->last_save_ms <= 0)
        return 1;
    return now_ms() - e->last_save_ms >= e->save_interval_ms;
}

/*
 * orig: 未叠加的原始帧；mask: 纯调色板掩码；vis: 画好 overlay 的帧。
 * 按 test.py / infer_board.c 的方式横向拼成 input | mask | overlay 三联图，
 * JPEG 存盘。目录满时先删最旧的一张。
 */
static double seg_save_dirty(SegEngine *e, const uint8_t *orig,
                             const uint8_t *mask, const uint8_t *vis, int w,
                             int h, const InferInfo *info, const char *src)
{
    if (!mask || !seg_save_due(e))
        return 0.0;
    /* 限速：脏污可能持续很久，逐帧保存三联图会占满 CPU/eMMC，推流变卡 */
    const double t_start = now_ms();
    e->last_save_ms = t_start;
    seg_trim_dir(e->save_dir, e->save_max);

    /* 与 test.py / infer_board.c 一致：input | mask | overlay 三联图 */
    const int cw = w * 3;
    uint8_t *canvas = (uint8_t *)malloc((size_t)cw * h * 3);
    if (!canvas) {
        logmsg("内存不足，跳过脏污对比图保存");
        return now_ms() - t_start;
    }
    for (int y = 0; y < h; ++y) {
        uint8_t *dst = canvas + (size_t)y * cw * 3;
        memcpy(dst, orig + (size_t)y * w * 3, (size_t)w * 3);
        memcpy(dst + (size_t)w * 3, mask + (size_t)y * w * 3, (size_t)w * 3);
        memcpy(dst + (size_t)w * 6, vis + (size_t)y * w * 3, (size_t)w * 3);
    }
    /* 面板名放左下角，避开 overlay 面板左上角的 LENS DIRTY 横幅 */
    seg_draw_text(canvas, cw, h, 10, h - 26, "input", 2);
    seg_draw_text(canvas, cw, h, w + 10, h - 26, "mask", 2);
    seg_draw_text(canvas, cw, h, w * 2 + 10, h - 26, "overlay", 2);
    Buffer enc = {0};
    int ok = encode_jpeg(canvas, cw, h, 85, &enc) == 0 && enc.data && enc.len > 0;
    free(canvas);
    if (!ok) {
        free(enc.data);
        logmsg("脏污对比图 JPEG 编码失败");
        return now_ms() - t_start;
    }

    char stem[NAME_MAX + 1];
    path_stem(src, stem, sizeof(stem));
    char path[PATH_MAX], tmp[PATH_MAX];
    if (snprintf(path, sizeof(path), "%s/dirty_%s.jpg", e->save_dir, stem) >=
            (int)sizeof(path) ||
        snprintf(tmp, sizeof(tmp), "%s.tmp", path) >= (int)sizeof(tmp)) {
        free(enc.data);
        return now_ms() - t_start;
    }

    int wrote = 0;
    FILE *f = fopen(tmp, "wb");
    if (f) {
        wrote = fwrite(enc.data, 1, enc.len, f) == enc.len;
        if (fclose(f) != 0)
            wrote = 0;
    }
    free(enc.data);
    if (!wrote || rename(tmp, path) != 0) {
        unlink(tmp);
        return now_ms() - t_start;
    }

    e->saved_n++;
    const double t = now_ms();
    if (t - e->last_save_log >= 5000.0) {
        e->last_save_log = t;
        logmsg("脏污对比图 %ld 张（上限 %d，间隔 %d ms）: %s (三类合计 %.1f%%)",
               e->saved_n, e->save_max, e->save_interval_ms, path, info->fg_ratio);
    }
    return t - t_start;
}

/* 跑一帧分割；dirty 时直接把 overlay 画进 rgb（mask_out 可空） */
static int seg_process(SegEngine *e, uint8_t *rgb, int w, int h, InferInfo *info,
                       uint8_t *mask_out, SegGeom *gout)
{
    const double t0 = now_ms();
    SegGeom g;
    uint8_t *model_rgb = seg_preprocess(rgb, w, h, e->mw, e->mh, &g);
    if (!model_rgb)
        return -1;
    seg_rgb_to_nv12(model_rgb, e->mw, e->mh, e->nv12);
    free(model_rgb);
    if (seg_infer(e, e->nv12, e->mw, e->mh, e->labels) != 0)
        return -1;

    const long px = (long)e->mw * e->mh;
    long cnt[SEG_MAX_CLASSES] = {0};
    for (long i = 0; i < px; ++i) {
        const uint8_t lab = e->labels[i];
        if (lab < e->n_class)
            cnt[lab]++;
    }
    /* 除 0 号 clean 外全部算脏污类，判定用合计，避免只盯一个类别漏检 */
    long fg = 0;
    for (int k = 1; k < e->n_class; ++k)
        fg += cnt[k];
    const long hit = cnt[e->target_class];
    memset(info, 0, sizeof(*info));
    info->enabled = 1;
    info->n_class = e->n_class;
    info->target_class = e->target_class;
    info->target_px = hit;
    info->target_ratio = (double)hit * 100.0 / (double)px;
    info->fg_ratio = (double)fg * 100.0 / (double)px;
    for (int k = 0; k < e->n_class; ++k)
        info->cls_ratio[k] = (double)cnt[k] * 100.0 / (double)px;
    info->dirty = info->fg_ratio >= e->dirty_ratio;
    seg_find_regions(e, &g, info);
    info->infer_ms = now_ms() - t0;

    if (info->dirty)
        seg_overlay(e->labels, &g, rgb, w, h, e->n_class, info, mask_out);
    if (gout)
        *gout = g;
    return 0;
}

/* 用 2px 蓝色实线框出实际送入检测网络的 640x384 范围（左右两列各一个框）
 * base 指向该列左上角，stride 是整张画布的宽度（并排图里右列不是连续存放的） */
static void draw_detect_box(uint8_t *base, int stride, int w, int h, int x0,
                            int y0, int x1, int y1)
{
    static const uint8_t blue[3] = {0, 255, 255}; /* 参考图同款青蓝色 */
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > w) x1 = w;
    if (y1 > h) y1 = h;
    if (x1 - x0 < 2 || y1 - y0 < 2)
        return;
    for (int t = 0; t < 2; ++t) {
        uint8_t *top = base + (size_t)(y0 + t) * stride * 3;
        uint8_t *bot = base + (size_t)(y1 - 1 - t) * stride * 3;
        for (int x = x0; x < x1; ++x) {
            memcpy(top + (size_t)x * 3, blue, 3);
            memcpy(bot + (size_t)x * 3, blue, 3);
        }
        for (int y = y0; y < y1; ++y) {
            uint8_t *row = base + (size_t)y * stride * 3;
            memcpy(row + (size_t)(x0 + t) * 3, blue, 3);
            memcpy(row + (size_t)(x1 - 1 - t) * 3, blue, 3);
        }
    }
}

/*
 * 按区域取平均缩小 RGB 图（最近邻会有锯齿，文字/蓝框很难看）。
 * 只在推流前用：先画好全尺寸画布再缩小，JPEG 字节数按面积比例掉，
 * 板端 CPU（A55）编码 1282x448 要 ~50 ms，缩到 640 宽只要 ~13 ms。
 */
static uint8_t *downscale_rgb(const uint8_t *src, int sw, int sh, int dw,
                              int dh)
{
    if (dw <= 0 || dh <= 0 || dw > sw || dh > sh)
        return NULL;
    uint8_t *dst = (uint8_t *)malloc((size_t)dw * dh * 3);
    if (!dst)
        return NULL;
    for (int y = 0; y < dh; ++y) {
        int y0 = (int)((long long)y * sh / dh);
        int y1 = (int)((long long)(y + 1) * sh / dh);
        if (y1 <= y0) y1 = y0 + 1;
        if (y1 > sh) y1 = sh;
        uint8_t *drow = dst + (size_t)y * dw * 3;
        for (int x = 0; x < dw; ++x) {
            int x0 = (int)((long long)x * sw / dw);
            int x1 = (int)((long long)(x + 1) * sw / dw);
            if (x1 <= x0) x1 = x0 + 1;
            if (x1 > sw) x1 = sw;
            unsigned a[3] = {0, 0, 0};
            for (int sy = y0; sy < y1; ++sy) {
                const uint8_t *srow = src + (size_t)sy * sw * 3;
                for (int sx = x0; sx < x1; ++sx) {
                    const uint8_t *p = srow + (size_t)sx * 3;
                    a[0] += p[0];
                    a[1] += p[1];
                    a[2] += p[2];
                }
            }
            const unsigned n = (unsigned)((y1 - y0) * (x1 - x0));
            uint8_t *d = drow + (size_t)x * 3;
            d[0] = (uint8_t)(a[0] / n);
            d[1] = (uint8_t)(a[1] / n);
            d[2] = (uint8_t)(a[2] / n);
        }
    }
    return dst;
}

/* 网页开启检测时的画面：左原图 | 2px 间隔 | 右检测图，两半都标出检测范围。
 * pub_width > 0 且小于画布宽度时先缩小再编码，pub_w/pub_h 返回实际尺寸。 */
static int compose_side_by_side(const uint8_t *orig, const uint8_t *det, int w,
                                int h, const SegGeom *g, int quality,
                                int pub_width, int *pub_w, int *pub_h,
                                Buffer *out)
{
    const int gap = 2;
    const int cw = w * 2 + gap;
    uint8_t *canvas = (uint8_t *)calloc((size_t)cw * h, 3);
    if (!canvas)
        return -1;
    for (int y = 0; y < h; ++y) {
        uint8_t *d = canvas + (size_t)y * cw * 3;
        memcpy(d, orig + (size_t)y * w * 3, (size_t)w * 3);
        memcpy(d + (size_t)(w + gap) * 3, det + (size_t)y * w * 3,
               (size_t)w * 3);
    }
    if (g->scale > 0.0 && g->cw > 0 && g->ch > 0) {
        int x0 = (int)(g->x0 / g->scale + 0.5);
        int x1 = (int)((g->x0 + g->cw) / g->scale + 0.5);
        int y0 = (int)(g->y0 / g->scale + 0.5);
        int y1 = (int)((g->y0 + g->ch) / g->scale + 0.5);
        draw_detect_box(canvas, cw, w, h, x0, y0, x1, y1);
        draw_detect_box(canvas + (size_t)(w + gap) * 3, cw, w, h, x0, y0, x1,
                        y1);
    }
    int rc;
    if (pub_width > 0 && pub_width < cw) {
        int dh = (int)((long long)h * pub_width / cw);
        if (dh < 1)
            dh = 1;
        uint8_t *small = downscale_rgb(canvas, cw, h, pub_width, dh);
        free(canvas);
        if (!small)
            return -1;
        rc = encode_jpeg(small, pub_width, dh, quality, out);
        free(small);
        *pub_w = pub_width;
        *pub_h = dh;
    } else {
        rc = encode_jpeg(canvas, cw, h, quality, out);
        free(canvas);
        *pub_w = cw;
        *pub_h = h;
    }
    return rc;
}

/*
 * 每帧分阶段耗时：只在 watch 线程里累加（无锁），每 10 s 打一行平均值。
 * 排延迟问题时先看这一行，确认瓶颈在解码、检测、推流编码还是存图。
 */
static struct {
    double sum_decode, sum_seg, sum_enc, sum_save;
    long n;
    double last_log;
} g_perf;

static void perf_add(double decode_ms, double seg_ms, double enc_ms,
                     double save_ms)
{
    g_perf.sum_decode += decode_ms;
    g_perf.sum_seg += seg_ms;
    g_perf.sum_enc += enc_ms;
    g_perf.sum_save += save_ms;
    g_perf.n++;
    const double now = now_ms();
    if (g_perf.last_log <= 0) {
        g_perf.last_log = now;
        return;
    }
    if (now - g_perf.last_log < 10000.0)
        return;
    const double n = (double)g_perf.n;
    logmsg("每帧耗时(ms): 解码 %.1f | 检测 %.1f | 推流编码 %.1f | 存图 %.1f"
           "（%ld 帧 / %.0f s）",
           g_perf.sum_decode / n, g_perf.sum_seg / n, g_perf.sum_enc / n,
           g_perf.sum_save / n, g_perf.n, (now - g_perf.last_log) / 1000.0);
    memset(&g_perf, 0, sizeof(g_perf));
    g_perf.last_log = now;
}

static int process_png(const char *path, FrameHub *hub, Buffer *enc,
                       int quality, int consume, SegEngine *seg)
{
    int w = 0, h = 0, n = 0;
    int rc = -1;
    const double tp0 = now_ms();
    uint8_t *rgb = stbi_load(path, &w, &h, &n, 3);
    const double tp1 = now_ms();
    if (!rgb) {
        logmsg("PNG 解码失败: %s (%s)", path, stbi_failure_reason());
    } else {
        InferInfo info;
        memset(&info, 0, sizeof(info));
        int infer_ok = 0;
        SegGeom geom;
        memset(&geom, 0, sizeof(geom));
        /* 网页开关关掉时彻底不做推理：不预处理、不叠加、不存对比图 */
        const int infer_on =
            seg && __atomic_load_n(&hub->infer_on, __ATOMIC_RELAXED);
        uint8_t *orig = NULL, *mask = NULL;
        /* 网页并排画面要用原图；出现脏污时拷三联图也要原图/掩码 */
        if (infer_on) {
            orig = (uint8_t *)malloc((size_t)w * h * 3);
            if (orig)
                memcpy(orig, rgb, (size_t)w * h * 3);
            /*
             * 掩码只用于对比图；限速下大部分帧根本不会存，省掉每帧 860 KB 的
             * calloc 和逐像素掩码写入（裁切区外补黑）。
             */
            if (seg_save_due(seg))
                mask = (uint8_t *)calloc((size_t)w * h, 3);
        }
        /* -a/-b 录制：overlay 会改写 rgb，先留一份原始帧 */
        uint8_t *rec_rgb = NULL;
        Buffer rec_orig = {0}, rec_vis = {0};
        int rec = 0, rec_vis_copy = 0;
        char rec_stem[NAME_MAX + 1] = {0};
        if (infer_on && seg_record_ready(seg)) {
            rec_rgb = (uint8_t *)malloc((size_t)w * h * 3);
            if (rec_rgb) {
                memcpy(rec_rgb, rgb, (size_t)w * h * 3);
                rec = 1;
                path_stem(path, rec_stem, sizeof(rec_stem));
            }
        }
        /* 数据采集：网页“保存”用叠加前的原始帧 */
        hub_set_raw(hub, rgb, w, h);
        double seg_ms = 0.0, save_ms = 0.0;
        if (infer_on) {
            const double ts = now_ms();
            int seg_ok = seg_process(seg, rgb, w, h, &info, mask, &geom) == 0;
            seg_ms = now_ms() - ts;
            if (seg_ok) {
                infer_ok = 1;
                if (info.dirty && orig && mask) {
                    const char *base = strrchr(path, '/');
                    save_ms = seg_save_dirty(seg, orig, mask, rgb, w, h, &info,
                                             base ? base + 1 : path);
                }
                if (rec && (!seg->rec_dirty_only || info.dirty)) {
                    if (encode_jpeg(rec_rgb, w, h, 85, &rec_orig) != 0) {
                        free(rec_orig.data);
                        rec_orig.data = NULL;
                    }
                    if (info.dirty) {
                        if (encode_jpeg(rgb, w, h, 85, &rec_vis) != 0) {
                            free(rec_vis.data);
                            rec_vis.data = NULL;
                        }
                    } else if (rec_orig.data) {
                        /* 未叠加时检测图与原图相同，直接复用编码结果 */
                        rec_vis = rec_orig;
                        rec_vis_copy = 1;
                    }
                    if (rec_orig.data && rec_vis.data)
                        seg_record_frame(seg, rec_stem, &rec_orig, &rec_vis);
                }
            } else {
                static int fail_logged;
                if (!fail_logged) {
                    fail_logged = 1;
                    logmsg("分割推理失败，后续帧继续推流（不再重复报错）");
                }
            }
        }
        free(mask);
        free(rec_rgb);
        free(rec_orig.data);
        if (!rec_vis_copy)
            free(rec_vis.data);
        /* 开启检测时推「左原图 | 右检测图」，用青框标出 640x384 检测范围 */
        int enc_ok;
        int pub_w = w, pub_h = h;
        /* 每帧读取当前宽度上限：网页 /res 切换后下一帧即生效 */
        const int pub_width = __atomic_load_n(&g_pub_width, __ATOMIC_RELAXED);
        const double te0 = now_ms();
        if (infer_ok && orig &&
            compose_side_by_side(orig, rgb, w, h, &geom, quality, pub_width,
                                 &pub_w, &pub_h, enc) == 0) {
            enc_ok = 0;
        } else if (pub_width > 0 && pub_width < w) {
            /* 未开检测时也统一按 --pub-width 缩，节省链路带宽 */
            int dh = (int)((long long)h * pub_width / w);
            if (dh < 1)
                dh = 1;
            uint8_t *small = downscale_rgb(rgb, w, h, pub_width, dh);
            enc_ok = small ? encode_jpeg(small, pub_width, dh, quality, enc)
                           : encode_jpeg(rgb, w, h, quality, enc);
            if (small) {
                free(small);
                pub_w = pub_width;
                pub_h = dh;
            }
        } else {
            enc_ok = encode_jpeg(rgb, w, h, quality, enc);
        }
        const double enc_ms = now_ms() - te0;
        perf_add(tp1 - tp0, seg_ms, enc_ms, save_ms);
        free(orig);
        if (enc_ok != 0) {
            logmsg("JPEG 编码失败: %s", path);
        } else {
            const char *base = strrchr(path, '/');
            hub_publish(hub, enc->data, enc->len, pub_w, pub_h,
                        base ? base + 1 : path, infer_ok ? &info : NULL);
            rc = 0;
        }
        stbi_image_free(rgb);
    }
    /* --consume：不管这一帧成没成功，都不留在本地 */
    if (consume)
        unlink(path);
    return rc;
}

/* 清空目录里已有的 *.png（--purge，用于清理历史积压） */
static void purge_pngs(const char *dir)
{
    DIR *d = opendir(dir);
    if (!d)
        return;
    unsigned long n = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (!has_png_ext(e->d_name))
            continue;
        char path[PATH_MAX];
        if (snprintf(path, sizeof(path), "%s/%s", dir, e->d_name) >=
            (int)sizeof(path))
            continue;
        if (unlink(path) == 0)
            n++;
    }
    closedir(d);
    logmsg("已删除目录内 %lu 张旧 PNG", n);
}

static int watch_loop(const char *dir, FrameHub *hub, int fps, int quality,
                      int consume, int purge, SegEngine *seg)
{
    if (purge)
        purge_pngs(dir);

    int ifd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (ifd < 0) {
        logmsg("inotify_init1 失败: %s", strerror(errno));
        return -1;
    }
    int wd = inotify_add_watch(ifd, dir, IN_CLOSE_WRITE | IN_MOVED_TO);
    if (wd < 0) {
        logmsg("无法监听目录 %s: %s（目录需先由 sub_img_imu 创建）",
               dir, strerror(errno));
        close(ifd);
        return -1;
    }
    logmsg("监听目录: %s", dir);

    Buffer enc = {0};
    double min_interval = fps > 0 ? 1000.0 / fps : 0.0;
    double last_pub = 0.0;
    char path[PATH_MAX];

    /* 先送一张已有图，避免页面空白（--consume 时一并删掉） */
    if (newest_png(dir, path, sizeof(path)) == 0) {
        logmsg("预热: %s", path);
        if (process_png(path, hub, &enc, quality, consume, seg) == 0)
            last_pub = now_ms();
    }

    char evbuf[8192] __attribute__((aligned(__alignof__(struct inotify_event))));
    struct pollfd pfd;
    pfd.fd = ifd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    while (!g_stop) {
        int rc = poll(&pfd, 1, 200);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (rc == 0)
            continue;
        ssize_t len = read(ifd, evbuf, sizeof(evbuf));
        if (len <= 0)
            continue;

        /*
         * 只认这一批事件里 mtime 最新的一张：解码+编码比源帧率慢时，
         * 逐张处理会让内核 inotify 队列越积越多，延迟无上界地增长。
         * 其余还没处理就过期的一律跳过（--consume 下直接删除）。
         */
        char best[NAME_MAX + 1] = {0};
        struct stat bst, st;
        int have_best = 0, overflow = 0;
        memset(&bst, 0, sizeof(bst));

        for (char *p = evbuf; p < evbuf + len;) {
            struct inotify_event *ev = (struct inotify_event *)p;
            p += sizeof(struct inotify_event) + ev->len;

            if (ev->mask & IN_Q_OVERFLOW) {
                overflow = 1;
                continue;
            }
            if (!(ev->mask & (IN_CLOSE_WRITE | IN_MOVED_TO)) || ev->len == 0)
                continue;
            if (!has_png_ext(ev->name))
                continue;

            if (snprintf(path, sizeof(path), "%s/%s", dir, ev->name) >=
                (int)sizeof(path))
                continue;
            if (stat(path, &st) != 0)
                continue;          /* 已被删掉或在别处消费 */

            if (have_best &&
                (st.st_mtim.tv_sec < bst.st_mtim.tv_sec ||
                 (st.st_mtim.tv_sec == bst.st_mtim.tv_sec &&
                  st.st_mtim.tv_nsec <= bst.st_mtim.tv_nsec))) {
                if (consume)
                    unlink(path);  /* 比当前最新的还旧，丢弃 */
                continue;
            }
            if (consume && have_best) {
                char old[PATH_MAX];
                snprintf(old, sizeof(old), "%s/%s", dir, best);
                unlink(old);
            }
            snprintf(best, sizeof(best), "%s", ev->name);
            bst = st;
            have_best = 1;
        }

        if (overflow) {
            /* 队列溢出（丢了事件）：直接扫目录取最新一张 */
            if (newest_png(dir, path, sizeof(path)) == 0) {
                last_pub = now_ms();
                process_png(path, hub, &enc, quality, consume, seg);
            }
            continue;
        }
        if (!have_best)
            continue;

        double t = now_ms();
        if (min_interval > 0 && last_pub > 0 && t - last_pub < min_interval) {
            /* 超过帧率上限：这一张不推；--consume 时也删掉，避免堆积 */
            if (consume) {
                snprintf(path, sizeof(path), "%s/%s", dir, best);
                unlink(path);
            }
            continue;
        }
        if (snprintf(path, sizeof(path), "%s/%s", dir, best) >= (int)sizeof(path))
            continue;
        last_pub = t;
        process_png(path, hub, &enc, quality, consume, seg);
    }

    free(enc.data);
    inotify_rm_watch(ifd, wd);
    close(ifd);
    return 0;
}

/* ------------------------------------------------- 帧源 2：直连相机（SDK） */

/*
 * MoTypes.h 里的 MoFrame 布局（不能直接 include：MoLocalSdkInterface.h
 * 无条件包含 <vector>，是 C++ 头）。这里按字段顺序复刻。
 */
typedef struct {
    unsigned long long TimeStamp;
    unsigned long long FrameId;
    unsigned int StreamType;
    unsigned int SensorId;
    unsigned short ResolutionWidth;
    unsigned short ResolutionHeight;
    unsigned int BitsPerPixel;
    unsigned int BytesUsed;
    int FillLightStatus;
    int FillLightBrightness;
    int RGBImageType;
    float exp_time;
    float analog_gain;
    uint8_t Reserved[64];
    void *Data;
} MoFrameC;

typedef struct {
    int (*moSetConfigPath)(const char *);
    int (*moOpenCamera)(int *);
    int (*moCloseCamera)(int);
    int (*moGetOneRGBDFrame)(int, MoFrameC *, MoFrameC *, MoFrameC *);
    int (*moReleaseRGBDFrame)(int, MoFrameC *, MoFrameC *, MoFrameC *);
    int (*moGetSdkVersion)(char *);
    char *(*moGetErrorString)(int);
} MoApi;

static int mo_resolve(MoApi *api, const char *lib)
{
    const char *tries[8];
    int n = 0;
    if (lib && *lib)
        tries[n++] = lib;
    tries[n++] = "libMoGeneralSDK.so.1.4";
    tries[n++] = "/goko/lib/libMoGeneralSDK.so.1.4";
    tries[n++] = "/goko/lib/libMoGeneralSDK.so";
    tries[n++] = "/app/data/metoak/lib/libMoGeneralSDK.so.1.4";
    tries[n++] = "/app/data/metoak/lib/libMoGeneralSDK.so";

    void *h = NULL;
    const char *last_err = NULL;
    for (int i = 0; i < n; i++) {
        h = dlopen(tries[i], RTLD_NOW | RTLD_GLOBAL);
        if (h) {
            logmsg("已加载 SDK: %s", tries[i]);
            break;
        }
        last_err = dlerror();
    }
    if (!h) {
        logmsg("无法加载 libMoGeneralSDK: %s", last_err ? last_err : "未知错误");
        logmsg("提示: 可设置 LD_LIBRARY_PATH=/goko/lib:/app/data/metoak/"
               "3rdparty_libs/opencv_libs:/app/data/metoak/3rdparty_libs/metoak_libs");
        return -1;
    }

    memset(api, 0, sizeof(*api));
#define MO_SYM(field, name)                                         \
    do {                                                            \
        void *sym = dlsym(h, name);                                 \
        if (!sym) {                                                 \
            logmsg("SDK 缺少符号 %s", name);                        \
            return -1;                                              \
        }                                                           \
        *(void **)(&api->field) = sym;                              \
    } while (0)
    MO_SYM(moSetConfigPath, "moSetConfigPath");
    MO_SYM(moOpenCamera, "moOpenCamera");
    MO_SYM(moCloseCamera, "moCloseCamera");
    MO_SYM(moGetOneRGBDFrame, "moGetOneRGBDFrame");
    MO_SYM(moReleaseRGBDFrame, "moReleaseRGBDFrame");
#undef MO_SYM
    api->moGetSdkVersion = (int (*)(char *))dlsym(h, "moGetSdkVersion");
    api->moGetErrorString = (char *(*)(int))dlsym(h, "moGetErrorString");
    return 0;
}

static inline uint8_t clip_u8(int v)
{
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

/* yuv420p（I420 或 NV12）→ packed RGB，BT.601 video range */
static void yuv_to_rgb(const uint8_t *yuv, int w, int h, int nv12, uint8_t *rgb)
{
    size_t ysz = (size_t)w * (size_t)h;
    const uint8_t *U = yuv + ysz;
    const uint8_t *V = U + (size_t)(w / 2) * (size_t)(h / 2);
    for (int y = 0; y < h; y++) {
        const uint8_t *yr = yuv + (size_t)y * (size_t)w;
        for (int x = 0; x < w; x++) {
            int Yv = yr[x];
            int Uv, Vv;
            if (nv12) {
                const uint8_t *uv = U + (size_t)(y / 2) * (size_t)w + (size_t)(x & ~1);
                Uv = uv[0];
                Vv = uv[1];
            } else {
                size_t ci = (size_t)(y / 2) * (size_t)(w / 2) + (size_t)(x / 2);
                Uv = U[ci];
                Vv = V[ci];
            }
            int c = Yv - 16;
            int d = Uv - 128;
            int e = Vv - 128;
            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;
            uint8_t *o = rgb + ((size_t)y * (size_t)w + (size_t)x) * 3;
            o[0] = clip_u8(r);
            o[1] = clip_u8(g);
            o[2] = clip_u8(b);
        }
    }
}

static int camera_loop(const char *cfg, const char *sdk_lib, int right_side,
                       int nv12, FrameHub *hub, int fps, int quality)
{
    MoApi api;
    if (mo_resolve(&api, sdk_lib) != 0)
        return -1;

    char version[128] = {0};
    if (api.moGetSdkVersion)
        api.moGetSdkVersion(version);

    /* 未指定 --cfg 时在常见位置自动找；都找不到就沿用 SDK 内置默认
     * ./config/x5_camera.cfg（相对当前工作目录）。 */
    const char *use_cfg = cfg;
    if (!use_cfg || !*use_cfg) {
        static const char *cands[] = {
            "/goko/config/S316_config/x5_camera.cfg",
            "/app/data/metoak/config/x5_camera.cfg",
        };
        for (size_t i = 0; i < sizeof(cands) / sizeof(cands[0]); i++) {
            if (access(cands[i], R_OK) == 0) {
                use_cfg = cands[i];
                break;
            }
        }
    }
    if (use_cfg && *use_cfg) {
        int rc = api.moSetConfigPath(use_cfg);
        logmsg("moSetConfigPath(%s) -> %d%s%s", use_cfg, rc,
               api.moGetErrorString ? " " : "",
               (api.moGetErrorString && rc != 0) ? api.moGetErrorString(rc) : "");
    } else {
        logmsg("未找到相机配置，SDK 将使用默认 ./config/x5_camera.cfg"
               "（相对当前目录，找不到请用 --cfg 指定）");
    }

    int handle = -1;
    int rc = api.moOpenCamera(&handle);
    if (rc != 0) {
        logmsg("moOpenCamera 失败: %d %s", rc,
               api.moGetErrorString ? api.moGetErrorString(rc) : "");
        logmsg("若 sensor 进程正在占用模组，先停掉它再试；或改用 --watch 模式");
        return -1;
    }
    logmsg("相机已打开, SDK 版本 %s, 使用%s目", version[0] ? version : "?",
           right_side ? "右" : "左");

    Buffer enc = {0};
    int fail = 0;
    double last_warn = 0.0;
    double last_pub = 0.0;
    double min_interval = fps > 0 ? 1000.0 / fps : 0.0;

    while (!g_stop) {
        MoFrameC left = {0}, right = {0}, disp = {0};
        rc = api.moGetOneRGBDFrame(handle, &left, &right, &disp);
        if (rc != 0) {
            if (++fail == 1 || now_ms() - last_warn > 1000.0) {
                last_warn = now_ms();
                logmsg("取帧失败 rc=%d %s (连续 %d 次)", rc,
                       api.moGetErrorString ? api.moGetErrorString(rc) : "",
                       fail);
            }
            usleep(20000);
            continue;
        }
        fail = 0;

        MoFrameC *f = right_side ? &right : &left;
        int w = f->ResolutionWidth;
        int h = f->ResolutionHeight;
        const uint8_t *data = (const uint8_t *)f->Data;
        if (!data || w <= 0 || h <= 0 || (w & 1) || (h & 1)) {
            api.moReleaseRGBDFrame(handle, &left, &right, &disp);
            usleep(5000);
            continue;
        }
        size_t yuv_need = (size_t)w * (size_t)h +
                          2 * ((size_t)(w / 2) * (size_t)(h / 2));
        if (f->BytesUsed && (size_t)f->BytesUsed < yuv_need) {
            if (now_ms() - last_warn > 1000.0) {
                last_warn = now_ms();
                logmsg("帧数据不完整: %ux%u BytesUsed=%u < %zu",
                       (unsigned)w, (unsigned)h, f->BytesUsed, yuv_need);
            }
            api.moReleaseRGBDFrame(handle, &left, &right, &disp);
            usleep(5000);
            continue;
        }

        double t = now_ms();
        if (min_interval > 0 && last_pub > 0 && t - last_pub < min_interval) {
            api.moReleaseRGBDFrame(handle, &left, &right, &disp);
            usleep(2000);
            continue;
        }

        size_t need = (size_t)w * (size_t)h * 3;
        uint8_t *rgb = malloc(need);
        if (!rgb) {
            api.moReleaseRGBDFrame(handle, &left, &right, &disp);
            usleep(10000);
            continue;
        }
        yuv_to_rgb(data, w, h, nv12, rgb);
        hub_set_raw(hub, rgb, w, h); /* 数据采集：网页“保存”用原始帧 */
        int ok = encode_jpeg(rgb, w, h, quality, &enc);
        free(rgb);
        api.moReleaseRGBDFrame(handle, &left, &right, &disp);

        if (ok == 0) {
            last_pub = t;
            hub_publish(hub, enc.data, enc.len, w, h,
                        right_side ? "camera:right" : "camera:left", NULL);
        } else {
            logmsg("JPEG 编码失败 (%dx%d)", w, h);
        }
    }

    free(enc.data);
    api.moCloseCamera(handle);
    return 0;
}

/* ----------------------------------------------------------------- main */

static int all_digits(const char *s)
{
    if (!s || !*s)
        return 0;
    for (; *s; ++s)
        if (*s < '0' || *s > '9')
            return 0;
    return 1;
}

static void usage(FILE *f)
{
    fprintf(f,
        "用法:\n"
        "  %s --watch DIR [选项]     # 监听目录中新出现的 PNG（配合 sub_img_imu）\n"
        "  %s --camera [选项]        # 直接打开 S316 双目模组取流（dlopen Metoak SDK）\n"
        "\n"
        "帧源:\n"
        "  --watch DIR        监听目录中的 *.png（IN_CLOSE_WRITE），例如\n"
        "                     /userdata/mono_orig/image_data/stereo_left\n"
        "  --camera           通过 libMoGeneralSDK 直接取 RGBD 帧（640x544 yuv420p）\n"
        "  --cfg PATH         相机配置文件（默认自动找 /goko/config/S316_config/、\n"
        "                     /app/data/metoak/config/ 下的 x5_camera.cfg）\n"
        "  --sdk-lib PATH     指定 libMoGeneralSDK.so 路径（默认自动查找）\n"
        "  --right            相机模式取右目（默认左目）\n"
        "  --yuv LAYOUT       相机帧 YUV 布局: i420（默认）或 nv12\n"
        "  --consume          watch 模式处理完删除 PNG（避免 eMMC 写满）\n"
        "  --purge            watch 启动时先删掉目录里已有的全部 PNG（清历史积压）\n"
        "\n"
        "脏污检测（可选，仅 --watch 模式）:\n"
        "  --model PATH       分割模型（如 model.bin）。开启后每帧推理，\n"
        "                     0 号类 clean 不叠加，其余脏污类按调色板颜色\n"
        "                     50%% 混合；合计占比超过阈值时显示 LENS DIRTY。\n"
        "                     网页上的「是否开启脏污检测」可随时开关实时推理\n"
        "  --labels PATH      类别名文件（默认找模型同目录的 labels.txt，\n"
        "                     每行一类，第一行是 clean）\n"
        "  --target-class N   主目标类别（默认 %d，只影响 target_ratio）\n"
        "  --dirty-ratio PCT  判定脏污的所有脏污类（class>=1）合计面积占比\n"
        "                     阈值，单位 %%（默认 %.2f）\n"
        "  --region-min-px N  脏污连通域最小面积（模型像素，默认 %d），更小的\n"
        "                     当噪点丢掉；网页/状态最多给出 %d 个连通域\n"
        "  --save-dir PATH    脏污帧保存目录（默认 %s）：把「原图 | 掩码 |\n"
        "                     叠加图」三联图横向拼成一张 JPG（同 test.py）\n"
        "  --save-max N       保存目录内最多保留 N 张，超出删 mtime 最旧的一张；\n"
        "                     0 表示不保存（默认 %d）\n"
        "  --save-interval MS 两张对比图的最小间隔（ms），0 = 每脏污帧都存\n"
        "                     （默认 %d；脏污持续时按间隔抽帧，避免卡顿）\n"
        "  -a [N], --record-all[=N]\n"
        "                     实时检测录制：每一帧都存原图和检测图\n"
        "                     （orig_<帧名>.jpg / det_<帧名>.jpg），只保留最新\n"
        "                     N 帧，超出删最旧（N 默认 %d；需要 --model）\n"
        "  -b [N], --record-dirty[=N]\n"
        "                     只录制被判脏污的帧（同样存原图+检测图），只保留\n"
        "                     最新 N 帧（默认 %d）；用于回看干净镜头下的误检\n"
        "  --record-dir PATH  录制目录（默认 %s）\n"
        "\n"
        "推流:\n"
        "  --port N           HTTP 端口（默认 %d）\n"
        "  --fps N            帧率上限，0 表示不限（默认 %d）\n"
        "  --quality N        JPEG 质量 1-100（默认 %d）\n"
        "  --pub-width N      推流画面宽度上限（默认 %d = 不缩）。开启检测时\n"
        "                     并排画面 1282 宽、每帧 ~90 KB，Wi-Fi 差的链路\n"
        "                     会积压；给 640 可把字节数降到约 1/4。运行时也\n"
        "                     可在网页「推流宽度」下拉框调整（GET /res?w=N）\n"
        "  -h, --help         显示本帮助\n"
        "\n"
        "示例:\n"
        "  /goko/app/sub_img_imu 2 &\n"
        "  ./stream_server --watch /userdata/mono_orig/image_data/stereo_left \\\n"
        "      --consume --purge\n"
        "  # 带脏污检测（labels.txt 自动取模型同目录）:\n"
        "  ./stream_server --watch /userdata/mono_orig/image_data/stereo_left \\\n"
        "      --consume --purge --model ./models/model.bin\n"
        "  # 浏览器打开 http://<板子IP>:%d/\n"
        "\n"
        "  LD_LIBRARY_PATH=/goko/lib:/app/data/metoak/3rdparty_libs/opencv_libs:"
        "/app/data/metoak/3rdparty_libs/metoak_libs \\\n"
        "    ./stream_server --camera\n",
        PROG, PROG, DEFAULT_TARGET_CLASS, DEFAULT_DIRTY_RATIO,
        SEG_REGION_MIN_PX, SEG_MAX_REGIONS,
        DEFAULT_SAVE_DIR, DEFAULT_SAVE_MAX, DEFAULT_SAVE_INTERVAL_MS,
        DEFAULT_RECORD_MAX,
        DEFAULT_RECORD_MAX, DEFAULT_RECORD_DIR,
        DEFAULT_PORT, DEFAULT_FPS, DEFAULT_QUALITY, DEFAULT_PUB_WIDTH,
        DEFAULT_PORT);
}

int main(int argc, char **argv)
{
    const char *watch_dir = NULL;
    const char *cfg = NULL;
    const char *sdk_lib = NULL;
    const char *yuv = "i420";
    const char *model = NULL;
    const char *labels = NULL;
    int camera = 0, right_side = 0, consume = 0, purge = 0;
    int port = DEFAULT_PORT, fps = DEFAULT_FPS, quality = DEFAULT_QUALITY;
    int pub_width = DEFAULT_PUB_WIDTH;
    int target_class = DEFAULT_TARGET_CLASS;
    double dirty_ratio = DEFAULT_DIRTY_RATIO;
    int region_min_px = SEG_REGION_MIN_PX;
    const char *save_dir = DEFAULT_SAVE_DIR;
    int save_max = DEFAULT_SAVE_MAX;
    int save_interval_ms = DEFAULT_SAVE_INTERVAL_MS;
    const char *record_dir = DEFAULT_RECORD_DIR;
    int record_mode = 0; /* 0=关，'a'=每帧，'b'=仅脏污帧 */
    int record_max = DEFAULT_RECORD_MAX;

    static const struct option opts[] = {
        { "watch", required_argument, 0, 'w' },
        { "camera", no_argument, 0, 'c' },
        { "cfg", required_argument, 0, 'C' },
        { "sdk-lib", required_argument, 0, 'S' },
        { "right", no_argument, 0, 'R' },
        { "yuv", required_argument, 0, 'Y' },
        { "consume", no_argument, 0, 'k' },
        { "purge", no_argument, 0, 'P' },
        { "port", required_argument, 0, 'p' },
        { "fps", required_argument, 0, 'f' },
        { "quality", required_argument, 0, 'q' },
        { "pub-width", required_argument, 0, 'W' },
        { "model", required_argument, 0, 'm' },
        { "labels", required_argument, 0, 'L' },
        { "target-class", required_argument, 0, 'T' },
        { "dirty-ratio", required_argument, 0, 'D' },
        { "region-min-px", required_argument, 0, 'G' },
        { "save-dir", required_argument, 0, 'd' },
        { "save-max", required_argument, 0, 'M' },
        { "save-interval", required_argument, 0, 'i' },
        { "record-all", optional_argument, 0, 'a' },
        { "record-dirty", optional_argument, 0, 'b' },
        { "record-dir", required_argument, 0, 'A' },
        { "help", no_argument, 0, 'h' },
        { 0, 0, 0, 0 },
    };

    /*
     * getopt 的可选参数只认 "-a200" / "--record-all=200"，不认 "-a 200"。
     * 用户更常用空格写法，所以先手工把 -a/-b 及其后面的数字摘出来，
     * 剩下的参数再交给 getopt_long。
     */
    char **fargv = (char **)calloc((size_t)argc + 1, sizeof(char *));
    if (!fargv)
        return 1;
    int fargc = 0;
    fargv[fargc++] = argv[0];
    for (int i = 1; i < argc; ++i) {
        const char *s = argv[i];
        char mode = 0;
        const char *num = NULL;
        if (strcmp(s, "-a") == 0 || strcmp(s, "-b") == 0) {
            mode = s[1];
            if (i + 1 < argc && all_digits(argv[i + 1]))
                num = argv[++i];
        } else if (s[0] == '-' && (s[1] == 'a' || s[1] == 'b') &&
                   all_digits(s + 2)) {
            mode = s[1];
            num = s + 2;
        }
        if (mode) {
            if (record_mode && record_mode != mode) {
                fprintf(stderr, "%s: -a 和 -b 只能选一个\n", PROG);
                return 2;
            }
            record_mode = mode;
            if (num)
                record_max = atoi(num);
            continue;
        }
        fargv[fargc++] = argv[i];
    }
    fargv[fargc] = NULL;
    argc = fargc;
    argv = fargv;

    int c;
    while ((c = getopt_long(argc, argv, "w:cC:S:RY:kPp:f:q:W:m:L:T:D:G:d:M:i:A:h",
                            opts,
                            NULL)) != -1) {
        switch (c) {
        case 'w': watch_dir = optarg; break;
        case 'c': camera = 1; break;
        case 'C': cfg = optarg; break;
        case 'S': sdk_lib = optarg; break;
        case 'R': right_side = 1; break;
        case 'Y': yuv = optarg; break;
        case 'k': consume = 1; break;
        case 'P': purge = 1; break;
        case 'p': port = atoi(optarg); break;
        case 'f': fps = atoi(optarg); break;
        case 'q': quality = atoi(optarg); break;
        case 'W': pub_width = atoi(optarg); break;
        case 'm': model = optarg; break;
        case 'L': labels = optarg; break;
        case 'T': target_class = atoi(optarg); break;
        case 'D': dirty_ratio = atof(optarg); break;
        case 'G': region_min_px = atoi(optarg); break;
        case 'd': save_dir = optarg; break;
        case 'M': save_max = atoi(optarg); break;
        case 'i': save_interval_ms = atoi(optarg); break;
        case 'a':
            if (record_mode && record_mode != 'a') {
                fprintf(stderr, "%s: -a 和 -b 只能选一个\n", PROG);
                return 2;
            }
            record_mode = 'a';
            if (optarg)
                record_max = atoi(optarg);
            break;
        case 'b':
            if (record_mode && record_mode != 'b') {
                fprintf(stderr, "%s: -a 和 -b 只能选一个\n", PROG);
                return 2;
            }
            record_mode = 'b';
            if (optarg)
                record_max = atoi(optarg);
            break;
        case 'A': record_dir = optarg; break;
        case 'h': usage(stdout); return 0;
        default: usage(stderr); return 2;
        }
    }

    if (watch_dir && camera) {
        fprintf(stderr, "%s: --watch 和 --camera 只能选一个\n", PROG);
        return 2;
    }
    if (!watch_dir && !camera) {
        usage(stderr);
        return 2;
    }
    if (port <= 0 || port > 65535) {
        fprintf(stderr, "%s: 非法端口 %d\n", PROG, port);
        return 2;
    }
    if (quality < 1)
        quality = 1;
    if (quality > 100)
        quality = 100;
    pub_width_set(pub_width); /* 负数按“不缩”，过小按 64 */
    if (save_max < 0)
        save_max = 0; /* 负数按“不保存”处理 */
    if (save_interval_ms < 0)
        save_interval_ms = 0; /* 负数按“不限速”处理 */
    if (record_max < 0)
        record_max = 0;
    if (record_mode && !model) {
        fprintf(stderr, "%s: -a/-b 需要同时指定 --model\n", PROG);
        return 2;
    }

    int nv12;
    if (strcasecmp(yuv, "i420") == 0 || strcasecmp(yuv, "yuv420p") == 0)
        nv12 = 0;
    else if (strcasecmp(yuv, "nv12") == 0 || strcasecmp(yuv, "yuv420sp") == 0)
        nv12 = 1;
    else {
        fprintf(stderr, "%s: --yuv 只支持 i420 / nv12\n", PROG);
        return 2;
    }

    if (model) {
        struct stat st;
        if (camera) {
            fprintf(stderr, "%s: --model 目前只支持 --watch 模式\n", PROG);
            return 2;
        }
        if (stat(model, &st) != 0 || !S_ISREG(st.st_mode)) {
            fprintf(stderr, "%s: 模型不存在: %s\n", PROG, model);
            return 2;
        }
        if (target_class < 0 || target_class >= SEG_MAX_CLASSES) {
            /* 实际类别数由 labels.txt 决定，seg_open 里还会再校验一次 */
            fprintf(stderr, "%s: --target-class 必须在 0..%d 之间\n", PROG,
                    SEG_MAX_CLASSES - 1);
            return 2;
        }
        if (dirty_ratio < 0.0 || dirty_ratio > 100.0) {
            fprintf(stderr, "%s: --dirty-ratio 必须在 0..100 之间\n", PROG);
            return 2;
        }
        if (region_min_px < 0 || region_min_px > 100000) {
            fprintf(stderr, "%s: --region-min-px 必须在 0..100000 之间\n", PROG);
            return 2;
        }
    }

    signal(SIGPIPE, SIG_IGN);
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    FrameHub hub;
    hub_init(&hub);

    DataStore data;
    data_init(&data);

    SegEngine seg;
    SegEngine *segp = NULL;
    if (model) {
        if (seg_open(&seg, model, labels, target_class, dirty_ratio,
                     region_min_px, save_dir, save_max, save_interval_ms,
                     record_mode ? record_dir : NULL, record_max,
                     record_mode == 'b') != 0) {
            seg_close(&seg);
            hub_destroy(&hub);
            return 1;
        }
        segp = &seg;
        /* 网页“是否开启脏污检测”：默认开，可运行时关掉纯推流 */
        hub.has_model = 1;
        hub.infer_on = 1;
        hub.n_class = seg.n_class;
    }

    HttpServer srv;
    if (http_start(&srv, port, &hub, &data) != 0)
        return 1;

    logmsg("%s 启动: http://<板子IP>:%d/  (帧源: %s%s%s)", PROG, port,
           watch_dir ? "watch " : "camera ",
           watch_dir ? watch_dir : (right_side ? "right" : "left"),
           model ? ", 脏污检测已开启" : "");

    int rc;
    if (watch_dir)
        rc = watch_loop(watch_dir, &hub, fps, quality, consume, purge, segp);
    else
        rc = camera_loop(cfg, sdk_lib, right_side, nv12, &hub, fps, quality);

    hub.quit = 1;
    pthread_mutex_lock(&hub.mu);
    pthread_cond_broadcast(&hub.cv);
    pthread_mutex_unlock(&hub.mu);
    srv.stop = 1;
    if (srv.listen_fd >= 0)
        shutdown(srv.listen_fd, SHUT_RDWR);
    pthread_join(srv.tid, NULL);
    usleep(150000); /* 给 MJPEG 客户端线程收尾的时间 */

    logmsg("退出: 共推 %lu 帧", hub.frames);
    if (segp)
        seg_close(segp);
    hub_destroy(&hub);
    return rc == 0 ? 0 : 1;
}
