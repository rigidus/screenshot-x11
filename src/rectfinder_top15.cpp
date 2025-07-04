#include <opencv2/opencv.hpp>
#include <numeric>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <array>

static const char* PIPE = "/tmp/screenshot_pipe";

/* пороги */
constexpr int MIN_RECT_W = 1;
constexpr int MIN_RECT_H = 1;
constexpr int TOP_K      = 15;      // сколько корзин обрабатывать

/* ---------- LUT: 64 квантованных BGR цвета --------------------------- */
static cv::Vec3b LUT[64];
static void initLUT()
{
    for (int id = 0; id < 64; ++id) {
        int r2 = (id >> 4) & 3, g2 = (id >> 2) & 3, b2 = id & 3;
        LUT[id] = cv::Vec3b(b2 * 85, g2 * 85, r2 * 85);
    }
}

/* квантуем BGR → 6-бит id */
static inline uint8_t quant6(const cv::Vec3b& p)
{
    return ((p[2] >> 6) << 4) | ((p[1] >> 6) << 2) | (p[0] >> 6);
}

/* рисуем рамку противоположным цветом */
static void drawBBox(cv::Mat& img, const cv::Rect& rc, const cv::Vec3b& c)
{
    cv::Scalar inv(255 - c[0], 255 - c[1], 255 - c[2]);
    cv::rectangle(img, rc, inv, 2);
}

static void drawDoubleBBox(cv::Mat& img,
                           const cv::Rect& rc,
                           const cv::Vec3b& base,
                           int t = 1)                     // толщина каждой рамки
{
    cv::Scalar inv(255 - base[0], 255 - base[1], 255 - base[2]);
    cv::Scalar orig(base[0], base[1], base[2]);


    /* 1) внутренняя рамка — ровно по bounding-box */
    cv::rectangle(img, rc, /* inv */ cv::Vec3b(0, 0, 255), t);

    /* 2) внешняя рамка — rc, увеличенный на t px по всем сторонам */
    cv::Rect outer = rc;
    outer.x      -= t;
    outer.y      -= t;
    outer.width  += 2*t;
    outer.height += 2*t;

    /* обрезаем, чтобы не выйти за края картинки */
    outer &= cv::Rect(0, 0, img.cols, img.rows);

    cv::rectangle(img, outer, /* orig */ cv::Vec3b(0, 255, 255), t);
}


/* обработка одного PNG ------------------------------------------------- */
static void process(const std::string& inPath)
{
    cv::Mat src = cv::imread(inPath, cv::IMREAD_COLOR);
    if (src.empty()) { std::cerr << "cannot read " << inPath << '\n'; return; }

    const int W = src.cols, H = src.rows;

    /* шаг 1: строим карту 6-битных цветов + гистограмму 64 корзин */
    cv::Mat qmap(H, W, CV_8U);
    std::array<int, 64> hist{};        // = {0}
    for (int y = 0; y < H; ++y) {
        const cv::Vec3b* s = src.ptr<cv::Vec3b>(y);
        uint8_t*         d = qmap.ptr<uint8_t>(y);
        for (int x = 0; x < W; ++x) {
            uint8_t id = quant6(s[x]);
            d[x] = id;
            ++hist[id];
        }
    }

    /* шаг 2: выбираем TOP_K по убыванию встречаемости */
    std::vector<int> ids(64); std::iota(ids.begin(), ids.end(), 0);
    std::partial_sort(ids.begin(), ids.begin() + TOP_K, ids.end(),
					  [&](int a, int b) { return hist[a] > hist[b]; });

    /* если каких-то корзин нет вовсе -- сдвинем K вниз */
    int K = std::min<int>(TOP_K,
						  std::count_if(hist.begin(), hist.end(), [](int v){ return v>0; }));

    /* квантованная картинка (для визуализации) */
    cv::Mat qimg(H, W, CV_8UC3);
    for (int y = 0; y < H; ++y) {
        const uint8_t* id = qmap.ptr<uint8_t>(y);
        cv::Vec3b*     p  = qimg.ptr<cv::Vec3b>(y);
        for (int x = 0; x < W; ++x) p[x] = LUT[id[x]];
    }

    /* шаг 3: CCL только для top-K корзин */
    cv::Mat mask, labels, stats, centroids;
    for (int k = 0; k < K; ++k)
    {
        int id = ids[k];
        cv::compare(qmap, id, mask, cv::CmpTypes::CMP_EQ);
        if (hist[id] == 0) continue;           // перестраховка

        int n = cv::connectedComponentsWithStats(
			mask, labels, stats, centroids, 8,
			CV_32S, cv::CCL_GRANA);

        for (int i = 1; i < n; ++i)
        {
            int  x = stats.at<int>(i, cv::CC_STAT_LEFT);
            int  y = stats.at<int>(i, cv::CC_STAT_TOP);
            int  w = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int  h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            // if (w < MIN_RECT_W || h < MIN_RECT_H) continue;
            // drawBBox(qimg, cv::Rect(x, y, w, h), LUT[id]);
			drawDoubleBBox(qimg, cv::Rect(x, y, w, h), LUT[id]);
        }
    }

    /* шаг 4: сохраняем результат */
    std::string out = inPath;
    size_t pos = out.find_last_of('.');
    out.insert(pos == std::string::npos ? out.size() : pos, "_");
    cv::imwrite(out, qimg);
    std::cerr << "→ " << out << '\n';
}

/* --------------------------------------------------------------------- */
int main()
{
    initLUT();
    mkfifo(PIPE, 0666);
    int fd = open(PIPE, O_RDONLY);
    if (fd < 0) { perror("open fifo"); return 1; }
    FILE* fp = fdopen(fd, "r");
    if (!fp) { perror("fdopen"); return 1; }

    char buf[256];
    while (fgets(buf, sizeof buf, fp))
    {
        buf[strcspn(buf, "\n")] = 0;
        if (buf[0]) process(buf);
    }
    return 0;
}
