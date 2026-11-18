/*
 * min_dirty_infer.c — 最简版「单张图片/批量目录、单线程」脏污分割推理链路
 *
 * 来源图要求高 448（双目左目 PNG）或 384（模型输入尺寸）、宽 ≥ 640；
 * 多出的列左右各裁一半，高 448 时上下各裁 32 行。
 * 计时/阶段划分严格按下面三组 + 三个单列项：
 *
 *   预处理   = 解码 + 缩放裁切 + RGB→NV12
 *   纯推理   = hbDNNInfer 提交 + WaitTaskDone
 *   后处理   = label 解码/reshape + 统计
 *   （输入拷贝+flush、叠加、拼图、PNG 编码、写盘 分别单列，不计入上面三组）
 *
 * 数据流：
 *   stb 读图(640x448 / 640x384 / 642x448 等 RGB)
 *     → 左右各裁 (w-640)/2 列；高 448 时再上下各裁 32 行，得到 640x384（= 模型输入，无缩放）
 *     → RGB24 → packed NV12（BT.601 video range）
 *     → 拷入 hbDNN 输入张量 + flush
 *     → hbDNNInfer / WaitTaskDone
 *     → 输出 U8 [1,384,640,1] 逐像素类别 → labels[384*640]
 *     → 按 labels.txt 统计各类像素占比，class>=1 的脏污类合计判脏
 *     → 在原图上对前景做 50% 调色板叠加
 *     → 左原图 | 右叠加 横向拼成 2*src_w 宽，写出 PNG（可选）
 *
 * 构建: make（或 sh scripts/build.sh），产物 bin/min_dirty_infer
 * 运行: ./bin/min_dirty_infer [-i /path/图片或目录] [-o /path/输出或目录]
 *                          [-m /path/model.bin] [-l /path/labels.txt]
 *         默认输入目录 /userdata/iris/test_img，批量跑完打印各阶段平均值
 *
 * 仅支持 RDK X5 单输入 packed NV12、输出紧凑 uint8 单通道类别图的 .bin
 * （例如 02/output/model.bin）。通用场景请用 dirty_seg。
 */

#define _GNU_SOURCE

#include <dirent.h>
#include <dlfcn.h>
#include <getopt.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "hb_dnn.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#define STBI_ONLY_BMP
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/* ---- 来源图高 448（双目左目）或 384（已是模型输入尺寸），宽 ≥ 640，
 *      多出的列左右各裁一半；模型输入 640x384 ---- */
#define SRC_W 640
#define SRC_H 448 /* 双目左目 PNG 的高度 */
#define IN_W 640
#define IN_H 384
#define MAX_CLASSES 16              /* labels.txt 最多支持 16 类（0 = clean） */
#define DIRTY_RATIO 0.5             /* 所有脏污类合计面积占比阈值（%） */
#define DEFAULT_IMG_DIR "/userdata/iris/data_get"
#define DEFAULT_MODEL "/userdata/iris/models/model.bin"
#define CLASS_NAME_MAX 32

/* RGB 调色板：0 clean 不叠加；1 dirt_dot 红褐、2 glare 明黄、3 water 蓝、
 * 4 grass_residue 绿、5 mud_film 黄褐，其余按色相区分 */
static const uint8_t PALETTE_RGB[MAX_CLASSES][3] = {
    {0, 0, 0},       {165, 42, 42},   {255, 255, 0},   {0, 0, 255},
    {0, 200, 0},     {180, 120, 40},  {0, 200, 200},   {255, 128, 0},
    {140, 90, 255},  {255, 255, 255}, {128, 128, 128}, {90, 160, 90},
    {200, 160, 120}, {60, 120, 200},  {180, 180, 60},  {120, 60, 160},
};

static int g_n_class;                /* 实际类别数（由 labels.txt 决定） */
static char g_cls_name[MAX_CLASSES][CLASS_NAME_MAX];

/* labels.txt：每行一个类名，第一行 = clean。找不到时用内置 class0..class15。 */
static void load_labels(const char *labels_path, const char *model_path)
{
    const char *lp = labels_path;
    char cand[PATH_MAX];
    if (!lp) {
        const char *sl = strrchr(model_path, '/');
        if (sl) {
            int k = snprintf(cand, sizeof(cand), "%.*s/labels.txt",
                             (int)(sl - model_path), model_path);
            if (k > 0 && (size_t)k < sizeof(cand) && access(cand, R_OK) == 0)
                lp = cand;
        } else if (access("labels.txt", R_OK) == 0) {
            lp = "labels.txt";
        }
    }

    g_n_class = 0;
    if (lp && access(lp, R_OK) == 0) {
        FILE *f = fopen(lp, "r");
        if (f) {
            char line[256];
            while (g_n_class < MAX_CLASSES && fgets(line, sizeof(line), f)) {
                char *p = line;
                while (*p == ' ' || *p == '\t') ++p;
                char *e = p + strlen(p);
                while (e > p && (e[-1] == '\n' || e[-1] == '\r' ||
                                 e[-1] == ' ' || e[-1] == '\t'))
                    *--e = 0;
                if (!*p || *p == '#') continue;
                snprintf(g_cls_name[g_n_class], CLASS_NAME_MAX, "%s", p);
                g_n_class++;
            }
            fclose(f);
        }
        if (g_n_class > 0)
            printf("labels: %s（%d 类）\n", lp, g_n_class);
    }
    if (g_n_class <= 0) {
        g_n_class = MAX_CLASSES;
        printf("labels: 内置 class0..class%d（未找到 labels.txt）\n",
               MAX_CLASSES - 1);
    }
    for (int i = 0; i < g_n_class && i < MAX_CLASSES; ++i)
        if (!g_cls_name[i][0])
            snprintf(g_cls_name[i], CLASS_NAME_MAX, "class%d", i);
    for (int i = g_n_class; i < MAX_CLASSES; ++i)
        g_cls_name[i][0] = '\0';
}

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* 墙钟时间戳（用于标注各阶段耗时的完成时刻）：YYYY-mm-dd HH:MM:SS.mmm */
static void stamp(char *buf, size_t n)
{
    struct timespec ts;
    struct tm tm;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm);
    if (strftime(buf, n, "%Y-%m-%d %H:%M:%S", &tm) == 0) {
        buf[0] = '\0';
        return;
    }
    const size_t len = strlen(buf);
    snprintf(buf + len, n - len, ".%03ld", ts.tv_nsec / 1000000L);
}

#define DIE(...)                             \
    do {                                     \
        fprintf(stderr, "错误: " __VA_ARGS__); \
        fputc('\n', stderr);                 \
        exit(1);                             \
    } while (0)

/* ============================================================ 预处理部分 */

/* 1a. 解码：图片文件 -> RGB24。失败或尺寸不符返回 NULL（批量时跳过该图）。
 * 尺寸要求：高 448（上下各裁 32 行）或 384（已是模型输入）、宽 ≥ 640
 * 且多出的列数为偶数。 */
static uint8_t *decode_rgb(const char *path, int *out_w, int *out_h,
                           double *ms)
{
    double t0 = now_ms();
    int w = 0, h = 0, n = 0;
    uint8_t *rgb = stbi_load(path, &w, &h, &n, 3);
    *ms = now_ms() - t0;
    if (!rgb) {
        fprintf(stderr, "警告: 读图失败，跳过 %s (%s)\n", path,
                stbi_failure_reason());
        return NULL;
    }
    if ((h != SRC_H && h != IN_H) || w < IN_W || ((w - IN_W) & 1)) {
        fprintf(stderr,
                "警告: 跳过 %s：图片尺寸 %dx%d，要求高 %d 或 %d、宽 ≥ %d 且多出的列数为偶数\n",
                path, w, h, SRC_H, IN_H, IN_W);
        stbi_image_free(rgb);
        return NULL;
    }
    *out_w = w;
    *out_h = h;
    return rgb;
}

/* 1b. 裁切：(640+2k)x448 或 (640+2k)x384 -> 640x384。
 * 高 448 时上下各裁 32 行，高 384 时不裁；左右各裁 k 列，零拷贝（指针偏移）。 */
static const uint8_t *crop_to_model(const uint8_t *rgb, int src_w, int src_h,
                                    double *ms)
{
    double t0 = now_ms();
    const int x_off = (src_w - IN_W) / 2;
    const int y_off = (src_h - IN_H) / 2;
    const uint8_t *p = rgb + ((size_t)y_off * (size_t)src_w + (size_t)x_off) * 3;
    *ms = now_ms() - t0;
    return p;
}

/* 1c. 色彩空间：RGB24（行跨距 stride 字节）-> packed NV12（BT.601 video range） */
static void rgb_to_nv12(const uint8_t *rgb, int stride, int w, int h,
                        uint8_t *nv12, double *ms)
{
    double t0 = now_ms();
    uint8_t *yp = nv12;
    uint8_t *uv = nv12 + (size_t)w * h;

    for (int y = 0; y < h; ++y) {
        const uint8_t *row = rgb + (size_t)y * (size_t)stride;
        for (int x = 0; x < w; ++x) {
            int r = row[x * 3 + 0], g = row[x * 3 + 1], b = row[x * 3 + 2];
            int yv = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
            yp[(size_t)y * w + x] = (uint8_t)(yv < 16 ? 16 : yv > 235 ? 235 : yv);
        }
    }
    /* 与 cv2.COLOR_BGR2YUV_I420 一致：色度取 2x2 块左上角像素，不平均 */
    for (int y = 0; y < h / 2; ++y) {
        const uint8_t *row = rgb + (size_t)(2 * y) * (size_t)stride;
        for (int x = 0; x < w / 2; ++x) {
            int r = row[x * 2 * 3 + 0];
            int g = row[x * 2 * 3 + 1];
            int b = row[x * 2 * 3 + 2];
            int u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
            int v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
            uv[(size_t)y * w + x * 2 + 0] = (uint8_t)(u < 0 ? 0 : u > 255 ? 255 : u);
            uv[(size_t)y * w + x * 2 + 1] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
        }
    }
    *ms = now_ms() - t0;
}

/* ========================================================== libdnn / hbDNN */

typedef struct {
    void *lib;
    int32_t (*hbSysAllocCachedMem)(hbSysMem *, uint32_t);
    int32_t (*hbSysFlushMem)(hbSysMem *, int32_t);
    int32_t (*hbSysFreeMem)(hbSysMem *);
    int32_t (*hbDNNInitializeFromFiles)(hbPackedDNNHandle_t *, char const **, int32_t);
    int32_t (*hbDNNRelease)(hbPackedDNNHandle_t);
    int32_t (*hbDNNGetModelNameList)(char const ***, int32_t *, hbPackedDNNHandle_t);
    int32_t (*hbDNNGetModelHandle)(hbDNNHandle_t *, hbPackedDNNHandle_t, char const *);
    int32_t (*hbDNNGetInputTensorProperties)(hbDNNTensorProperties *, hbDNNHandle_t, int32_t);
    int32_t (*hbDNNGetOutputTensorProperties)(hbDNNTensorProperties *, hbDNNHandle_t, int32_t);
    int32_t (*hbDNNInfer)(hbDNNTaskHandle_t *, hbDNNTensor **, hbDNNTensor const *,
                          hbDNNHandle_t, hbDNNInferCtrlParam *);
    int32_t (*hbDNNWaitTaskDone)(hbDNNTaskHandle_t, int32_t);
    int32_t (*hbDNNReleaseTask)(hbDNNTaskHandle_t);

    hbPackedDNNHandle_t packed;
    hbDNNHandle_t handle;
    hbDNNTensorProperties in_props;
    hbDNNTensorProperties out_props;
    hbDNNTensor input;
    hbDNNTensor output;
} Engine;

#define LOAD_SYM(e, name)                                              \
    do {                                                               \
        *(void **)(&(e)->name) = dlsym((e)->lib, #name);               \
        if (!(e)->name) DIE("libdnn.so 缺少符号 %s", #name);           \
    } while (0)

static void engine_open(Engine *e, const char *model)
{
    memset(e, 0, sizeof(*e));

    /* 板端 libdnn 的依赖在 /usr/hobot/lib，不在默认搜索路径：先 GLOBAL 预加载 */
    static const char *const preload[] = {
        "/usr/hobot/lib/libalog.so.1", "/usr/hobot/lib/libhbmem.so.1",
        "/usr/hobot/lib/libcnn_intf.so.1"};
    for (size_t i = 0; i < sizeof(preload) / sizeof(preload[0]); ++i)
        if (access(preload[i], R_OK) == 0)
            (void)dlopen(preload[i], RTLD_NOW | RTLD_GLOBAL);

    static const char *const cands[] = {
        "libdnn.so", "libdnn.so.1", "/usr/lib/libdnn.so", "/usr/lib/libdnn.so.1"};
    for (size_t i = 0; i < sizeof(cands) / sizeof(cands[0]) && !e->lib; ++i)
        e->lib = dlopen(cands[i], RTLD_NOW | RTLD_GLOBAL);
    if (!e->lib)
        DIE("无法加载 libdnn.so: %s", dlerror());

    LOAD_SYM(e, hbSysAllocCachedMem);
    LOAD_SYM(e, hbSysFlushMem);
    LOAD_SYM(e, hbSysFreeMem);
    LOAD_SYM(e, hbDNNInitializeFromFiles);
    LOAD_SYM(e, hbDNNRelease);
    LOAD_SYM(e, hbDNNGetModelNameList);
    LOAD_SYM(e, hbDNNGetModelHandle);
    LOAD_SYM(e, hbDNNGetInputTensorProperties);
    LOAD_SYM(e, hbDNNGetOutputTensorProperties);
    LOAD_SYM(e, hbDNNInfer);
    LOAD_SYM(e, hbDNNWaitTaskDone);
    LOAD_SYM(e, hbDNNReleaseTask);

    char const *file = model;
    if (e->hbDNNInitializeFromFiles(&e->packed, &file, 1) != 0)
        DIE("加载模型失败: %s", model);

    char const **names = NULL;
    int32_t name_count = 0;
    if (e->hbDNNGetModelNameList(&names, &name_count, e->packed) != 0 ||
        name_count < 1)
        DIE("获取模型名失败");
    if (e->hbDNNGetModelHandle(&e->handle, e->packed, names[0]) != 0)
        DIE("获取模型句柄失败");

    if (e->hbDNNGetInputTensorProperties(&e->in_props, e->handle, 0) != 0)
        DIE("获取输入属性失败");
    if (e->hbDNNGetOutputTensorProperties(&e->out_props, e->handle, 0) != 0)
        DIE("获取输出属性失败");

    /* 本示例走紧凑布局，因此要求 aligned 尺寸正好等于有效尺寸 */
    const size_t in_bytes = (size_t)IN_W * IN_H * 3 / 2;
    const size_t out_bytes = (size_t)IN_W * IN_H;
    if (e->in_props.tensorType != HB_DNN_IMG_TYPE_NV12 ||
        (size_t)e->in_props.alignedByteSize != in_bytes)
        DIE("模型输入不是 %dx%d packed NV12（type=%d aligned=%d）", IN_W, IN_H,
            e->in_props.tensorType, e->in_props.alignedByteSize);
    if (e->out_props.tensorType != HB_DNN_TENSOR_TYPE_U8 ||
        (size_t)e->out_props.alignedByteSize != out_bytes)
        DIE("模型输出不是紧凑 uint8 类别图（type=%d aligned=%d）",
            e->out_props.tensorType, e->out_props.alignedByteSize);

    e->input.properties = e->in_props;
    if (e->hbSysAllocCachedMem(&e->input.sysMem[0],
                               (uint32_t)e->in_props.alignedByteSize) != 0)
        DIE("输入内存分配失败");
    memset(e->input.sysMem[0].virAddr, 0, (size_t)e->in_props.alignedByteSize);

    e->output.properties = e->out_props;
    if (e->hbSysAllocCachedMem(&e->output.sysMem[0],
                               (uint32_t)e->out_props.alignedByteSize) != 0)
        DIE("输出内存分配失败");

    printf("model %s: input NV12 %dx%d, output U8 [%d,%d]\n", names[0], IN_W,
           IN_H, IN_W, IN_H);
}

static void engine_close(Engine *e)
{
    if (e->output.sysMem[0].virAddr) e->hbSysFreeMem(&e->output.sysMem[0]);
    if (e->input.sysMem[0].virAddr) e->hbSysFreeMem(&e->input.sysMem[0]);
    if (e->packed) e->hbDNNRelease(e->packed);
    if (e->lib) dlclose(e->lib);
}

/* ================================================= 输入拷贝（单列，不算预处理） */

static void engine_fill_input(Engine *e, const uint8_t *nv12, double *ms)
{
    double t0 = now_ms();
    const size_t in_bytes = (size_t)IN_W * IN_H * 3 / 2;
    memcpy(e->input.sysMem[0].virAddr, nv12, in_bytes);
    if (e->hbSysFlushMem(&e->input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN) != 0)
        DIE("输入 flush 失败");
    *ms = now_ms() - t0;
}

/* ============================================================== 纯推理部分 */

/* hbDNNInfer 提交 + WaitTaskDone；提交与等待分别计时 */
static void engine_infer(Engine *e, double *t_submit, double *t_wait)
{
    hbDNNInferCtrlParam ctrl;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl);
    hbDNNTaskHandle_t task = NULL;
    hbDNNTensor *outs = &e->output;

    double t0 = now_ms();
    if (e->hbDNNInfer(&task, &outs, &e->input, e->handle, &ctrl) != 0)
        DIE("hbDNNInfer 失败");
    double t1 = now_ms();
    if (e->hbDNNWaitTaskDone(task, 0) != 0) {
        e->hbDNNReleaseTask(task);
        DIE("hbDNNWaitTaskDone 失败");
    }
    double t2 = now_ms();
    e->hbDNNReleaseTask(task);

    *t_submit = t1 - t0;
    *t_wait = t2 - t1;
}

/* ============================================================ 后处理部分 */

/* label 解码/reshape：invalidate cache 后把输出张量拷成 labels[384*640] */
static void decode_labels(Engine *e, uint8_t *labels, double *ms)
{
    double t0 = now_ms();
    e->hbSysFlushMem(&e->output.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    memcpy(labels, e->output.sysMem[0].virAddr, (size_t)IN_W * IN_H);
    *ms = now_ms() - t0;
}

/* 统计：各类占比 + 所有脏污类（class>=1）并集判脏 */
static void compute_stats(const uint8_t *labels, double *ms)
{
    double t0 = now_ms();
    long cnt[MAX_CLASSES] = {0};
    const long px = (long)IN_W * IN_H;
    for (long i = 0; i < px; ++i) {
        const int c = labels[i];
        if (c >= 0 && c < g_n_class) cnt[c]++;
    }
    long fg = 0;
    for (int k = 1; k < g_n_class; ++k) fg += cnt[k];
    *ms = now_ms() - t0;

    const double fg_ratio = (double)fg * 100.0 / (double)px;
    printf("classes:");
    for (int k = 0; k < g_n_class; ++k)
        printf(" %s=%.2f%%", g_cls_name[k],
               (double)cnt[k] * 100.0 / (double)px);
    printf("\nfg(class1..%d)=%.2f%% -> %s\n", g_n_class - 1, fg_ratio,
           fg_ratio >= DIRTY_RATIO ? "DIRTY" : "clean");
}

/* ============================================ 叠加 / 编码 / 写盘（单列） */

/* 在原图 src_w x src_h 上对前景做 50% 调色板叠加；裁切区外的行列保持原样 */
static void overlay_foreground(uint8_t *rgb, int src_w, int src_h, int x_off,
                               int y_off, const uint8_t *labels, double *ms)
{
    double t0 = now_ms();
    for (int y = 0; y < src_h; ++y) {
        const int my = y - y_off;
        if (my < 0 || my >= IN_H) continue;
        uint8_t *row = rgb + (size_t)y * (size_t)src_w * 3 + (size_t)x_off * 3;
        const uint8_t *lab = labels + (size_t)my * IN_W;
        for (int x = 0; x < IN_W; ++x) {
            const int c = lab[x];
            if (c <= 0 || c >= g_n_class) continue;
            uint8_t *p = row + (size_t)x * 3;
            p[0] = (uint8_t)((p[0] + PALETTE_RGB[c][0]) >> 1);
            p[1] = (uint8_t)((p[1] + PALETTE_RGB[c][1]) >> 1);
            p[2] = (uint8_t)((p[2] + PALETTE_RGB[c][2]) >> 1);
        }
    }
    *ms = now_ms() - t0;
}

/* PNG 编码（内存中）与写盘分开计时；同时给出两个阶段各自的完成时间戳 */
static void write_png(const char *path, const uint8_t *rgb, int w, int h,
                      double *t_enc, double *t_write, char *ts_enc,
                      size_t ts_enc_n, char *ts_write, size_t ts_write_n)
{
    double t0 = now_ms();
    int len = 0;
    unsigned char *png = stbi_write_png_to_mem(rgb, w * 3, w, h, 3, &len);
    double t1 = now_ms();
    stamp(ts_enc, ts_enc_n);
    if (!png) DIE("PNG 编码失败: %s", path);

    int ok = 0;
    FILE *f = fopen(path, "wb");
    if (f) {
        ok = fwrite(png, 1, (size_t)len, f) == (size_t)len;
        if (fclose(f) != 0) ok = 0;
    }
    STBIW_FREE(png);
    double t2 = now_ms();
    stamp(ts_write, ts_write_n);
    if (!ok) DIE("写 PNG 失败: %s", path);

    *t_enc = t1 - t0;
    *t_write = t2 - t1;
}

/* ------------------------------------------------------------------ 批量输入 */

/* 扩展名是否属于可解码图片（PNG/JPG/BMP，忽略大小写） */
static int is_image_name(const char *name)
{
    const char *dot = strrchr(name, '.');
    return dot && (!strcasecmp(dot, ".png") || !strcasecmp(dot, ".jpg") ||
                   !strcasecmp(dot, ".jpeg") || !strcasecmp(dot, ".bmp"));
}

static const char *base_name(const char *path)
{
    const char *s = strrchr(path, '/');
    return s ? s + 1 : path;
}

static int cmp_path(const void *a, const void *b)
{
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

/* 递归收集目录下所有图片（含子目录，最多 8 层，防符号链接环） */
static void collect_dir(const char *dir, int depth, char ***items, int *n,
                        int *cap)
{
    if (depth > 8) return;
    DIR *d = opendir(dir);
    if (!d) return;

    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (!strcmp(de->d_name, ".") || !strcmp(de->d_name, "..")) continue;
        char path[PATH_MAX];
        const int k = snprintf(path, sizeof(path), "%s/%s", dir, de->d_name);
        if (k < 0 || (size_t)k >= sizeof(path)) continue;

        struct stat st;
        if (stat(path, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            collect_dir(path, depth + 1, items, n, cap);
        } else if (S_ISREG(st.st_mode) && is_image_name(de->d_name)) {
            if (*n == *cap) {
                *cap = *cap ? *cap * 2 : 32;
                char **p = (char **)realloc(*items, (size_t)*cap * sizeof(*p));
                if (!p) DIE("内存不足");
                *items = p;
            }
            (*items)[(*n)++] = strdup(path);
            if (!(*items)[(*n) - 1]) DIE("内存不足");
        }
    }
    closedir(d);
}

/* 列出目录（递归）下所有图片，按路径排序；返回图片数，打不开返回 -1 */
static int collect_images(const char *dir, char ***out)
{
    DIR *d = opendir(dir);
    if (!d) return -1;
    closedir(d);

    char **items = NULL;
    int n = 0, cap = 0;
    collect_dir(dir, 0, &items, &n, &cap);
    if (n > 0) qsort(items, (size_t)n, sizeof(*items), cmp_path);
    *out = items;
    return n;
}

/* ------------------------------------------------------------------ 单图链路 */

typedef struct {
    double decode, crop, nv12, pre;
    double fill;
    double submit, wait, infer;
    double label, stats, post;
    double overlay, compose, enc, write;
    double total;
} Timings;

/* 跑一张图的完整链路并打印各阶段耗时；out 为 NULL 时不写图。
 * 返回 0 表示已处理，-1 表示该图不可用（已打印警告，调用方跳过）。 */
static int process_image(const char *img, const char *out, Engine *eng,
                         Timings *t)
{
    char ts_start[32], ts_pre[32], ts_fill[32], ts_infer[32], ts_post[32];
    char ts_overlay[32], ts_compose[32] = "", ts_enc[32] = "", ts_write[32] = "";
    memset(t, 0, sizeof(*t));

    stamp(ts_start, sizeof(ts_start));
    const double t_all = now_ms();

    /* ---- 预处理 = 解码 + 缩放裁切 + RGB→NV12 ---- */
    int src_w = 0, src_h = 0;
    uint8_t *rgb = decode_rgb(img, &src_w, &src_h, &t->decode);
    if (!rgb) return -1;
    /* -o 输出「左原图 | 右可视化」并排图；叠加会改写 rgb，先留一份原图 */
    uint8_t *orig = NULL;
    if (out) {
        orig = (uint8_t *)malloc((size_t)src_w * src_h * 3);
        if (!orig) DIE("内存不足");
        memcpy(orig, rgb, (size_t)src_w * src_h * 3);
    }
    const int x_off = (src_w - IN_W) / 2;
    const int y_off = (src_h - IN_H) / 2;
    const uint8_t *crop = crop_to_model(rgb, src_w, src_h, &t->crop);
    uint8_t *nv12 = (uint8_t *)malloc((size_t)IN_W * IN_H * 3 / 2);
    if (!nv12) DIE("内存不足");
    rgb_to_nv12(crop, src_w * 3, IN_W, IN_H, nv12, &t->nv12);
    t->pre = t->decode + t->crop + t->nv12;
    stamp(ts_pre, sizeof(ts_pre));

    /* ---- 输入拷贝 + flush（单列，不属于预处理/推理/后处理） ---- */
    engine_fill_input(eng, nv12, &t->fill);
    stamp(ts_fill, sizeof(ts_fill));

    /* ---- 纯推理 = 提交 + 等待 ---- */
    engine_infer(eng, &t->submit, &t->wait);
    t->infer = t->submit + t->wait;
    stamp(ts_infer, sizeof(ts_infer));

    /* ---- 后处理 = label 解码/reshape + 统计 ---- */
    uint8_t *labels = (uint8_t *)malloc((size_t)IN_W * IN_H);
    if (!labels) DIE("内存不足");
    decode_labels(eng, labels, &t->label);
    compute_stats(labels, &t->stats);
    t->post = t->label + t->stats;
    stamp(ts_post, sizeof(ts_post));

    /* ---- 叠加 / 拼图 / PNG 编码 / 写盘（单列） ---- */
    overlay_foreground(rgb, src_w, src_h, x_off, y_off, labels, &t->overlay);
    stamp(ts_overlay, sizeof(ts_overlay));
    if (out) {
        /* 左：原图；右：叠加图，横向拼接成一张 2*src_w 宽的 PNG */
        const int cw = src_w * 2;
        double tc0 = now_ms();
        uint8_t *canvas = (uint8_t *)malloc((size_t)cw * src_h * 3);
        if (!canvas) DIE("内存不足");
        for (int y = 0; y < src_h; ++y) {
            uint8_t *d = canvas + (size_t)y * cw * 3;
            memcpy(d, orig + (size_t)y * src_w * 3, (size_t)src_w * 3);
            memcpy(d + (size_t)src_w * 3, rgb + (size_t)y * src_w * 3,
                   (size_t)src_w * 3);
        }
        t->compose = now_ms() - tc0;
        stamp(ts_compose, sizeof(ts_compose));
        write_png(out, canvas, cw, src_h, &t->enc, &t->write, ts_enc,
                  sizeof(ts_enc), ts_write, sizeof(ts_write));
        free(canvas);
        free(orig);
        orig = NULL;
    }

    t->total = now_ms() - t_all;

    printf("--- 每图耗时 (开始 %s) ---\n", ts_start);
    printf("[预处理]  合计 %6.2f ms   (解码 %.2f + 裁剪 %.2f + RGB->NV12 %.2f) @ %s\n",
           t->pre, t->decode, t->crop, t->nv12, ts_pre);
    printf("[纯推理]  合计 %6.2f ms   (提交 %.2f + 等待 %.2f) @ %s\n", t->infer,
           t->submit, t->wait, ts_infer);
    printf("[后处理]  合计 %6.2f ms   (label 解码 %.2f + 统计 %.2f) @ %s\n", t->post,
           t->label, t->stats, ts_post);
    printf("[单列]    输入拷贝+flush %.2f ms @ %s\n", t->fill, ts_fill);
    printf("[单列]    叠加 %.2f ms @ %s\n", t->overlay, ts_overlay);
    if (out)
        printf("[单列]    拼图 %.2f ms @ %s | PNG 编码 %.2f ms @ %s | 写盘 %.2f ms @ %s\n",
               t->compose, ts_compose, t->enc, ts_enc, t->write, ts_write);
    else
        printf("[单列]    未开启输出\n");
    printf("total %.2f ms -> %s\n", t->total,
           out ? out : "(仅统计，未写图)");

    free(labels);
    free(nv12);
    stbi_image_free(rgb);
    return 0;
}

/* ------------------------------------------------------------------ main */

static void usage(FILE *f)
{
    fprintf(f,
            "用法: min_dirty_infer [-i <图片或目录>] [-o <输出>] "
            "[-m <模型>] [-l <labels>]\n"
            "\n"
            "  -i PATH   输入图片（高 448 或 384、宽 ≥ 640，多出的列左右各裁一半），\n"
            "            或包含图片的目录（递归扫描子目录，默认 %s）\n"
            "  -o PATH   输出「左原图 | 右叠加」并排 PNG；输入为目录时作为\n"
            "            输出目录，逐图写 vis_<原名>.png（省略则只打印统计）\n"
            "  -m FILE   .bin 模型（默认 %s）\n"
            "  -l FILE   类别名文件，每行一类，第一行是 clean\n"
            "            （默认找模型同目录的 labels.txt）\n"
            "  -h        显示帮助\n",
            DEFAULT_IMG_DIR, DEFAULT_MODEL);
}

int main(int argc, char **argv)
{
    const char *img = DEFAULT_IMG_DIR, *out = NULL;
    const char *model = DEFAULT_MODEL, *labels = NULL;
    int c;
    while ((c = getopt(argc, argv, "i:o:m:l:h")) != -1) {
        switch (c) {
        case 'i': img = optarg; break;
        case 'o': out = optarg; break;
        case 'm': model = optarg; break;
        case 'l': labels = optarg; break;
        case 'h': usage(stdout); return 0;
        default: usage(stderr); return 1;
        }
    }
    load_labels(labels, model);

    struct stat st;
    if (stat(img, &st) != 0) DIE("输入不存在: %s", img);
    const int is_dir = S_ISDIR(st.st_mode);

    char **files = NULL;
    int n_img = 0;
    if (is_dir) {
        n_img = collect_images(img, &files);
        if (n_img < 0) DIE("无法读取目录: %s", img);
        if (n_img == 0) DIE("目录中没有图片: %s", img);
    } else {
        files = (char **)malloc(sizeof(*files));
        if (!files) DIE("内存不足");
        files[0] = strdup(img);
        if (!files[0]) DIE("内存不足");
        n_img = 1;
    }

    /* 目录输入时 -o 视为输出目录，逐图写 vis_<原名>.png */
    char out_dir[PATH_MAX] = "";
    if (out && is_dir) {
        snprintf(out_dir, sizeof(out_dir), "%s", out);
        const size_t l = strlen(out_dir);
        if (l > 0 && out_dir[l - 1] != '/')
            strncat(out_dir, "/", sizeof(out_dir) - l - 1);
        (void)mkdir(out_dir, 0755); /* 已存在则忽略 */
    }

    /* ---- 模型加载：一次性，单独列出，不计入单图平均 ---- */
    const double t_open0 = now_ms();
    Engine eng;
    engine_open(&eng, model);
    const double t_open = now_ms() - t_open0;
    printf("model open: %.2f ms (一次性，不计入单图耗时)\n", t_open);

    Timings sum;
    memset(&sum, 0, sizeof(sum));
    int ok_n = 0, failed_n = 0;
    const double t_wall0 = now_ms();
    for (int i = 0; i < n_img; ++i) {
        printf("\n[%d/%d] %s\n", i + 1, n_img, base_name(files[i]));

        char out_path[PATH_MAX];
        const char *piece = NULL;
        if (out) {
            if (is_dir) {
                char stem[NAME_MAX + 1];
                snprintf(stem, sizeof(stem), "%s", base_name(files[i]));
                char *dot = strrchr(stem, '.');
                if (dot) *dot = '\0';
                const int k = snprintf(out_path, sizeof(out_path),
                                       "%svis_%s.png", out_dir, stem);
                if (k < 0 || (size_t)k >= sizeof(out_path))
                    DIE("输出路径过长: %s%s", out_dir, stem);
            } else {
                snprintf(out_path, sizeof(out_path), "%s", out);
            }
            piece = out_path;
        }

        Timings t;
        if (process_image(files[i], piece, &eng, &t) != 0) {
            failed_n++;
            continue;
        }
        ok_n++;
        sum.decode += t.decode;
        sum.crop += t.crop;
        sum.nv12 += t.nv12;
        sum.pre += t.pre;
        sum.fill += t.fill;
        sum.submit += t.submit;
        sum.wait += t.wait;
        sum.infer += t.infer;
        sum.label += t.label;
        sum.stats += t.stats;
        sum.post += t.post;
        sum.overlay += t.overlay;
        sum.compose += t.compose;
        sum.enc += t.enc;
        sum.write += t.write;
        sum.total += t.total;
    }
    const double t_wall = now_ms() - t_wall0;

    if (ok_n == 0) DIE("没有可处理的图片（失败 %d 张）", failed_n);
    if (failed_n > 0)
        fprintf(stderr,
                "\n警告: %d 张图片被跳过（读图失败，或高不是 %d/%d、宽不是 ≥%d 偶数余列）\n",
                failed_n, SRC_H, IN_H, IN_W);

    /* ---- 平均值汇总（只统计处理成功的图片） ---- */
    const double n = (double)ok_n;
    printf("\n=== 平均耗时 (%d 张成功 / %d 张) ===\n", ok_n, n_img);
    printf("[一次性]  模型加载 %.2f ms（不在平均值内）\n", t_open);
    printf("[预处理]  平均 %6.2f ms   (解码 %.2f + 裁剪 %.2f + RGB->NV12 %.2f)\n",
           sum.pre / n, sum.decode / n, sum.crop / n, sum.nv12 / n);
    printf("[纯推理]  平均 %6.2f ms   (提交 %.2f + 等待 %.2f)\n", sum.infer / n,
           sum.submit / n, sum.wait / n);
    printf("[后处理]  平均 %6.2f ms   (label 解码 %.2f + 统计 %.2f)\n",
           sum.post / n, sum.label / n, sum.stats / n);
    printf("[单列]    输入拷贝+flush 平均 %.2f ms\n", sum.fill / n);
    printf("[单列]    叠加 平均 %.2f ms", sum.overlay / n);
    if (out)
        printf(" | 拼图 平均 %.2f ms | PNG 编码 平均 %.2f ms | 写盘 平均 %.2f ms\n",
               sum.compose / n, sum.enc / n, sum.write / n);
    else
        printf(" | 未开启输出\n");
    printf("平均每图 total %.2f ms；本批 %d 张共 %.2f s (%.2f 张/s)\n",
           sum.total / n, ok_n, t_wall / 1000.0,
           ok_n / (t_wall > 1e-6 ? t_wall / 1000.0 : 1e-6));

    for (int i = 0; i < n_img; ++i) free(files[i]);
    free(files);
    engine_close(&eng);
    return 0;
}

   