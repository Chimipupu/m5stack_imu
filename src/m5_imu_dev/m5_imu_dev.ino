#include <M5StickCPlus.h>
#include <stdint.h>
#include <math.h>

#define LCD_HIGHT                        135
#define LCD_WIDTH                        320
#define LCD_ROTATION_PORTRAIT            0   // 縦向き（標準）
#define LCD_ROTATION_LANDSCAPE           1   // 横向き（右回転）
#define LCD_ROTATION_PORTRAIT_INV        2   // 縦向き逆さま
#define LCD_ROTATION_LANDSCAPE_INV       3   // 横向き逆さま

#define ACCEL_OFFSET_AVG_CNT              100
#define ACCEL_AVG_NUM                     100

#define ACC_GRABITY                       0.98 // 重力加速度の1/10

typedef struct {
    float q;   // プロセスノイズ共分散
    float r;   // 観測ノイズ共分散
    float x;   // 推定値
    float p;   // 推定誤差共分散
    float k;   // カルマンゲイン
} KalmanFilter;

KalmanFilter g_kf_x, g_kf_y, g_kf_z;        // カルマンフィルタの構造体
TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);  // LCDのスプライト

float g_acc_x = 0.0;          // 加速度センサのX軸の加速度
float g_acc_y = 0.0;          // 加速度センサのY軸の加速度
float g_acc_z = 0.0;          // 加速度センサのZ軸の加速度

float g_acc_x_offset = 0.0;   // 加速度センサのX軸のオフセット
float g_acc_y_offset = 0.0;   // 加速度センサのY軸のオフセット
float g_acc_z_offset = 0.0;   // 加速度センサのZ軸のオフセット

bool g_acc_top_axis_x = false; // 加速度センサのX軸が上向きか（重力加速度がかかる軸）のフラグ
bool g_acc_top_axis_y = false; // 加速度センサのY軸が上向きか（重力加速度がかかる軸）のフラグ
bool g_acc_top_axis_z = false; // 加速度センサのZ軸が上向きか（重力加速度がかかる軸）のフラグ

float g_vel_x = 0.0;    // 速度 X軸
float g_vel_y = 0.0;    // 速度 Y軸
float g_vel_z = 0.0;    // 速度 Z軸
unsigned long g_last_time = 0;

static void get_acc_offset_remove_val(float *p_acc, float *p_offset);

static void get_acc_offset_remove_val(float *p_acc, float *p_offset)
{
    *p_acc = (*p_acc - (*p_offset / 10));
}

void kalman_filter_init(KalmanFilter *p_kf, float process_noise, float measurement_noise, float initial_value)
{
    p_kf->q = process_noise;
    p_kf->r = measurement_noise;
    p_kf->x = initial_value;
    p_kf->p = 1.0f;
    p_kf->k = 0.0f;
}

float kalman_filter_update(KalmanFilter *p_kf, float measurement)
{
    // 予測ステップ
    p_kf->p = p_kf->p + p_kf->q;

    // 更新ステップ
    p_kf->k = p_kf->p / (p_kf->p + p_kf->r);
    p_kf->x = p_kf->x + p_kf->k * (measurement - p_kf->x);
    p_kf->p = (1.0f - p_kf->k) * p_kf->p;

    return p_kf->x;
}

void kalman_filter_reset(KalmanFilter *p_kf, float val)
{
    p_kf->x = val;
    p_kf->p = 1.0f;
}

void acc_calibrate(float *p_acc_x_offset, float *p_acc_y_offset, float *p_acc_z_offset)
{
    uint32_t cnt;

    g_acc_x_offset = 0.0;
    g_acc_y_offset = 0.0;
    g_acc_z_offset = 0.0;
    g_acc_top_axis_x = false;
    g_acc_top_axis_y = false;
    g_acc_top_axis_z = false;

    sprite.fillSprite(TFT_BLACK);
    sprite.setCursor(0, 0);
    sprite.println("[Acc Calibrate]");

    float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    float acc_x, acc_y, acc_z;

    for (cnt = 0; cnt < ACCEL_OFFSET_AVG_CNT; cnt++)
    {
        M5.IMU.getAccelData(&acc_x, &acc_y, &acc_z);
        sum_x += acc_x;
        sum_y += acc_y;
        sum_z += acc_z;
    }

    float avg_x = sum_x / ACCEL_OFFSET_AVG_CNT;
    float avg_y = sum_y / ACCEL_OFFSET_AVG_CNT;
    float avg_z = sum_z / ACCEL_OFFSET_AVG_CNT;

    // 重力加速度がかかっている軸の検出
    float abs_x = fabs(avg_x);
    float abs_y = fabs(avg_y);
    float abs_z = fabs(avg_z);

    // どの軸に重力加速度がかかっているかを特定
    g_acc_top_axis_x = (abs_x > abs_y) && (abs_x > abs_z);
    g_acc_top_axis_y = (abs_y > abs_x) && (abs_y > abs_z);
    g_acc_top_axis_z = (abs_z > abs_x) && (abs_z > abs_y);

    // 該当軸から重力加速度を差し引いてオフセットとする
    if (g_acc_top_axis_x)
        avg_x -= copysign(ACC_GRABITY, avg_x);
    if (g_acc_top_axis_y)
        avg_y -= copysign(ACC_GRABITY, avg_y);
    if (g_acc_top_axis_z)
        avg_z -= copysign(ACC_GRABITY, avg_z);

    *p_acc_x_offset = (avg_x * 10);
    *p_acc_y_offset = (avg_y * 10);
    *p_acc_z_offset = (avg_z * 10);

    sprite.printf("Acc X ofs:%6.3f\n", *p_acc_x_offset);
    sprite.printf("Acc Y ofs:%6.3f\n", *p_acc_y_offset);
    sprite.printf("Acc Z ofs:%6.3f\n", *p_acc_z_offset);
    sprite.println("Finish");
    sprite.pushSprite(0, 0);
}

void setup()
{
    M5.begin();
    g_last_time = millis();

    // LCD初期化
    M5.Lcd.setRotation(LCD_ROTATION_LANDSCAPE);
    sprite.createSprite(LCD_WIDTH, LCD_HIGHT);
    sprite.setTextSize(2);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.fillSprite(TFT_BLACK);
    sprite.setCursor(0, 0);
    sprite.pushSprite(0, 0);

    // 加速度センサをキャリブレーション
    M5.IMU.Init();
    acc_calibrate(&g_acc_x_offset, &g_acc_y_offset, &g_acc_z_offset);

    // カルマンフィルタ初期化
    kalman_filter_init(&g_kf_x, 0.01f, 0.1f, 0.0f);
    kalman_filter_init(&g_kf_y, 0.01f, 0.1f, 0.0f);
    kalman_filter_init(&g_kf_z, 0.01f, 0.1f, 0.0f);

    delay(3000);
}

void loop()
{
    uint32_t cnt;
    float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    float acc_x, acc_y, acc_z;

    M5.update();

    // センサ値を複数回取得して合計
    for (cnt = 0; cnt < ACCEL_AVG_NUM; cnt++)
    {
        M5.IMU.getAccelData(&acc_x, &acc_y, &acc_z);
        get_acc_offset_remove_val(&acc_x, &g_acc_x_offset);
        get_acc_offset_remove_val(&acc_y, &g_acc_y_offset);
        get_acc_offset_remove_val(&acc_z, &g_acc_z_offset);
        sum_x += acc_x;
        sum_y += acc_y;
        sum_z += acc_z;
    }

    // 平均値を算出
    g_acc_x = (sum_x / ACCEL_AVG_NUM) * 10;
    g_acc_y = (sum_y / ACCEL_AVG_NUM) * 10;
    g_acc_z = (sum_z / ACCEL_AVG_NUM) * 10;

    // カルマンフィルタで平滑化
    g_acc_x = kalman_filter_update(&g_kf_x, g_acc_x);
    g_acc_y = kalman_filter_update(&g_kf_y, g_acc_y);
    g_acc_z = kalman_filter_update(&g_kf_z, g_acc_z);

    // 速度に変換
    unsigned long current_time = millis();
    float dt = (current_time - g_last_time) / 1000.0f; // 秒単位に変換
    g_last_time = current_time;

    // ドリフト除去
    float threshold = 0.05f;
    float acc_x_clamp = (fabs(g_acc_x) < threshold) ? 0.0f : g_acc_x;
    float acc_y_clamp = (fabs(g_acc_y) < threshold) ? 0.0f : g_acc_y;
    float acc_z_clamp = (fabs(g_acc_z) < threshold) ? 0.0f : g_acc_z;

    g_acc_x = acc_x_clamp;
    g_acc_y = acc_y_clamp;
    g_acc_z = acc_z_clamp;

    // 単純積分（加速度 [m/s^2] × dt = 速度 [m/s]）
    g_vel_x += g_acc_x * dt;
    g_vel_y += g_acc_y * dt;
    g_vel_z += g_acc_z * dt;

    // 表示
    sprite.fillSprite(TFT_BLACK);
    sprite.setCursor(0, 0);
    sprite.printf("Acc X:%6.3fm/s^2\n", g_acc_x);
    sprite.printf("Acc Y:%6.3fm/s^2\n", g_acc_y);
    sprite.printf("Acc Z:%6.3fm/s^2\n", g_acc_z);
    // sprite.printf("Acc X Ofs:%6.3f\n", g_acc_x_offset);
    // sprite.printf("Acc Y Ofs:%6.3f\n", g_acc_y_offset);
    // sprite.printf("Acc Z Ofs:%6.3f\n", g_acc_z_offset);
    sprite.printf("Vel X:%6.3fkm/h\n", g_vel_x);
    sprite.printf("Vel Y:%6.3fkm/h\n", g_vel_y);
    sprite.printf("Vel Z:%6.3fkm/h\n", g_vel_z);
    sprite.pushSprite(0, 0);

    delay(1000);
}