#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <string>

static const char* PIPE = "/tmp/screenshot_pipe";

constexpr int MIN_W = 5;         // минимальный bbox, чтобы показать
constexpr int MIN_H = 5;

/* ---------- LUT для 16-уровней серого: 0, 17, 34 … 255 --------------- */
static cv::Vec3b GLUT[16];
static void initLUT()
{
    for (int i = 0; i < 16; ++i) {
        uint8_t g = static_cast<uint8_t>(i * 17);     // 0..255
        GLUT[i] = cv::Vec3b(g, g, g);                 // B = G = R
    }
}

/* ---------- RGB → 4-битовая яркость (0..15) -------------------------- */
static inline uint8_t gray4(const cv::Vec3b& p)
{
    // integer-luma ≈ 0.299R + 0.587G + 0.114B
    int y = (p[2] * 77 + p[1] * 150 + p[0] * 29) >> 8;   // 0..255
    return static_cast<uint8_t>(y >> 4);                 // /16 → 0..15
}

/* ---------- рисуем рамку инвертированным серым ----------------------- */
static void drawBox(cv::Mat& img, const cv::Rect& rc, uint8_t grayId)
{
    uint8_t inv = 15 - grayId;                  // инверт. уровень
    cv::Scalar col(GLUT[inv]);                  // BGR
    cv::rectangle(img, rc, col, /*th=2*/ 2);
}

/* ---------- обработка одного PNG ------------------------------------ */
static void process(const std::string& path)
{
    cv::Mat src = cv::imread(path, cv::IMREAD_COLOR);
    if (src.empty()) { std::cerr << "bad read " << path << '\n'; return; }

    const int H = src.rows, W = src.cols;

    /* step-1: карта 4-бит яркости */
    cv::Mat gmap(H, W, CV_8U);
    for (int y = 0; y < H; ++y) {
        const auto* s = src.ptr<cv::Vec3b>(y);
        auto*       d = gmap.ptr<uint8_t>(y);
        for (int x = 0; x < W; ++x) d[x] = gray4(s[x]);
    }

    /* step-2: строим 16-уровневое изображение для вывода */
    cv::Mat grayImg(H, W, CV_8UC3);
    for (int y = 0; y < H; ++y) {
        const uint8_t* id = gmap.ptr<uint8_t>(y);
        auto*          gp = grayImg.ptr<cv::Vec3b>(y);
        for (int x = 0; x < W; ++x) gp[x] = GLUT[id[x]];
    }

    /* step-3: по каждому уровню – CCL + bbox */
    cv::Mat mask, labels, stats, centroids;
    for (uint8_t id = 0; id < 16; ++id) {
        cv::compare(gmap, id, mask, cv::CmpTypes::CMP_EQ);
        if (cv::countNonZero(mask) == 0) continue;

        int n = cv::connectedComponentsWithStats(
            mask, labels, stats, centroids, 8,
            CV_32S, cv::CCL_GRANA);

        for (int i = 1; i < n; ++i) {
            int x = stats.at<int>(i, cv::CC_STAT_LEFT);
            int y = stats.at<int>(i, cv::CC_STAT_TOP);
            int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            if (w < MIN_W || h < MIN_H) continue;
            drawBox(grayImg, {x, y, w, h}, id);
        }
    }

    /* step-4: сохраняем файл с подчёркиванием */
    std::string out = path;
    size_t p = out.find_last_of('.');
    out.insert(p == std::string::npos ? out.size() : p, "_");
    cv::imwrite(out, grayImg);
    std::cerr << "→ " << out << '\n';
}

/* ---------- main: читаем FIFO ---------------------------------------- */
int main()
{
    initLUT();
    mkfifo(PIPE, 0666);
    int fd = open(PIPE, O_RDONLY);
    if (fd < 0) { perror("fifo"); return 1; }
    FILE* fp = fdopen(fd, "r");
    if (!fp) { perror("fdopen"); return 1; }

    char buf[256];
    while (fgets(buf, sizeof buf, fp)) {
        buf[strcspn(buf, "\n")] = 0;
        if (buf[0]) process(buf);
    }
    return 0;
}
