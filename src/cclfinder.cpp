#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <string>

static const char* PIPE = "/tmp/screenshot_pipe";

/* пороги */
constexpr int MIN_RECT_W = 3;
constexpr int MIN_RECT_H = 3;

/* ---------- LUT: 64 квантованных BGR-цвета --------------------------- */
static cv::Vec3b LUT[64];
static void initLUT()
{
    for (int id = 0; id < 64; ++id) {
        int r2 = (id >> 4) & 3;   // R-канал (2 бита)
        int g2 = (id >> 2) & 3;   // G
        int b2 =  id       & 3;   // B
        LUT[id] = cv::Vec3b(b2 * 85, g2 * 85, r2 * 85);   // B-G-R порядок
    }
}

/* квантуем оригинальный BGR → 6-битный id 0…63 */
static inline uint8_t quant6(const cv::Vec3b& p)
{
    return ((p[2] >> 6) << 4) | ((p[1] >> 6) << 2) | (p[0] >> 6);
}

/* рисуем рамку противоположным цветом */
static void drawBBox(cv::Mat& img, const cv::Rect& rc, const cv::Vec3b& c)
{
    cv::Scalar inv(255 - c[0], 255 - c[1], 255 - c[2]);
    cv::rectangle(img, rc, inv, /*thickness=*/1);
}

/* обработка одного файла ---------------------------------------------- */
static void process(const std::string& inPath)
{
    cv::Mat src = cv::imread(inPath, cv::IMREAD_COLOR);
    if (src.empty()) { std::cerr << "cannot read " << inPath << '\n'; return; }

    const int W = src.cols, H = src.rows;

    /* шаг 1. карта 6-битных цветов */
    cv::Mat qmap(H, W, CV_8U);
    for (int y = 0; y < H; ++y) {
        const auto* s = src.ptr<cv::Vec3b>(y);
        auto*       d = qmap.ptr<uint8_t>(y);
        for (int x = 0; x < W; ++x) d[x] = quant6(s[x]);
    }

    /* шаг 2. делаем квантованную картинку в BGR-ной палитре */
    cv::Mat quantImg(H, W, CV_8UC3);
    for (int y = 0; y < H; ++y) {
        const uint8_t* id = qmap.ptr<uint8_t>(y);
        auto*          qp = quantImg.ptr<cv::Vec3b>(y);
        for (int x = 0; x < W; ++x) qp[x] = LUT[id[x]];
    }

    /* шаг 3. для каждой «корзины» цвета запускаем CCL */
    cv::Mat mask, labels, stats, centroids;
    for (uint8_t id = 0; id < 64; ++id)
    {
        cv::compare(qmap, id, mask, cv::CmpTypes::CMP_EQ);
        if (cv::countNonZero(mask) == 0) continue;

        int n = cv::connectedComponentsWithStats(
			mask, labels, stats, centroids, 8,
			CV_32S, cv::CCL_GRANA);

        for (int i = 1; i < n; ++i)                    // labels[0] = фон
        {
            int x = stats.at<int>(i, cv::CC_STAT_LEFT);
            int y = stats.at<int>(i, cv::CC_STAT_TOP);
            int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);

            if (w < MIN_RECT_W || h < MIN_RECT_H) continue;   // мелкие игнор

            drawBBox(quantImg, cv::Rect(x, y, w, h), LUT[id]);
        }
    }

    /* шаг 4. сохраняем с подчёркиванием */
    std::string out = inPath;
    size_t pos = out.find_last_of('.');
    out.insert(pos == std::string::npos ? out.size() : pos, "_");
    cv::imwrite(out, quantImg);
    std::cerr << "→ " << out << '\n';
}

/* main: слушаем FIFO ---------------------------------------------------- */
int main()
{
    initLUT();
    mkfifo(PIPE, 0666);
    int fd = open(PIPE, O_RDONLY);
    if (fd < 0) { perror("open fifo"); return 1; }
    FILE* fp = fdopen(fd, "r");
    if (!fp) { perror("fdopen"); return 1; }

    char buf[256];
    while (fgets(buf, sizeof buf, fp)) {
        buf[strcspn(buf, "\n")] = 0;
        if (buf[0]) process(buf);
    }
    return 0;
}
