#include <M5Core2.h>
#include <stdint.h>
#include <math.h>

#define ACCEL_OFFSET_AVG_CNT    1000
#define ACCEL_AVG_NUM           1000

// LCDのスプライト
TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);

// 重力加速度の1/10
#define ACC_GRABITY    0.98

float g_acc_x = 0.0;           // 加速度センサのX軸の加速度
float g_acc_y = 0.0;           // 加速度センサのY軸の加速度
float g_acc_z = 0.0;           // 加速度センサのZ軸の加速度

float g_acc_x_offset = 0.0;   // 加速度センサのX軸のオフセット
float g_acc_y_offset = 0.0;   // 加速度センサのY軸のオフセット
float g_acc_z_offset = 0.0;   // 加速度センサのZ軸のオフセット

bool g_acc_top_axis_x = false; // 加速度センサのX軸が上向きか（重力加速度がかかる軸）のフラグ
bool g_acc_top_axis_y = false; // 加速度センサのY軸が上向きか（重力加速度がかかる軸）のフラグ
bool g_acc_top_axis_z = false; // 加速度センサのZ軸が上向きか（重力加速度がかかる軸）のフラグ

static void get_acc_offset_remove_val(float *p_acc, float *p_offset);

static void get_acc_offset_remove_val(float *p_acc, float *p_offset)
{
    *p_acc = (*p_acc - (*p_offset / 10));
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
    sprite.println("[Accel Sensor Calibrate]");
    sprite.println("1. Place the device on a flat surface!");
    sprite.println("2. Don't touch the device!");
    sprite.println("3. Wait until the LCD indicates the Finish");

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

    sprite.printf("Accel X Offset: %6.3f\n", *p_acc_x_offset);
    sprite.printf("Accel Y Offset: %6.3f\n", *p_acc_y_offset);
    sprite.printf("Accel Z Offset: %6.3f\n", *p_acc_z_offset);
    sprite.println("Calibrate Finish!");
    sprite.pushSprite(0, 0);
}

void setup()
{
    M5.begin();
    M5.IMU.Init();

    sprite.createSprite(320, 240);
    sprite.setTextSize(2);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.fillSprite(TFT_BLACK);
    sprite.setCursor(0, 0);
    sprite.pushSprite(0, 0);

    acc_calibrate(&g_acc_x_offset, &g_acc_y_offset, &g_acc_z_offset);
    delay(3000);
}

void loop()
{
    uint32_t cnt;
    float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    float acc_x, acc_y, acc_z;

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

    // 表示
    sprite.fillSprite(TFT_BLACK);
    sprite.setCursor(0, 0);
    sprite.println("3-Axis Accel Develop");
    sprite.printf("Accel X: %6.3f [m/s^2]\n", g_acc_x);
    sprite.printf("Accel Y: %6.3f [m/s^2]\n", g_acc_y);
    sprite.printf("Accel Z: %6.3f [m/s^2]\n", g_acc_z);
    sprite.printf("Accel X Offset: %6.3f\n", g_acc_x_offset);
    sprite.printf("Accel Y Offset: %6.3f\n", g_acc_y_offset);
    sprite.printf("Accel Z Offset: %6.3f\n", g_acc_z_offset);
    sprite.pushSprite(0, 0);

    delay(1000);
}