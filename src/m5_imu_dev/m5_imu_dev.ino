#include <M5Core2.h>
#include <stdint.h>
#include <math.h>

#define ACCEL_OFFSET_AVG_CNT    100
#define ACCEL_AVG_NUM           100

// LCDのスプライト
TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);

// 重力加速度
#define ACC_GRABITY    0.98

float g_acc_x = 0.0;
float g_acc_y = 0.0;
float g_acc_z = 0.0;

float g_acc_x_offset = 0.0;
float g_acc_y_offset = 0.0;
float g_acc_z_offset = 0.0;

static void get_acc_offset_remove_val(float *p_acc, float *p_offset);

static void get_acc_offset_remove_val(float *p_acc, float *p_offset)
{
    *p_acc = (*p_acc - (*p_offset / 10));
}

void acc_calibrate(float *p_acc_x_offset, float *p_acc_y_offset, float *p_acc_z_offset)
{
    uint32_t cnt;

    sprite.fillSprite(TFT_BLACK);
    sprite.setCursor(0, 0);
    sprite.println("[Accel Sensor Calibrate]");
    sprite.println("1. Place the device on a flat surface!");
    sprite.println("2. Don't touch the device!");
    sprite.println("3. Wait until the LCD indicates the Finish");

    float sum_x_offset = 0.0, sum_y_offset = 0.0, sum_z_offset = 0.0;
    float acc_x_offset, acc_y_offset, acc_z_offset;

    // センサのオフセットを複数回取得して合計
    for (cnt = 0; cnt < ACCEL_OFFSET_AVG_CNT; cnt++)
    {
        M5.IMU.getAccelData(&acc_x_offset, &acc_y_offset, &acc_z_offset);
        sum_x_offset += acc_x_offset;
        sum_y_offset += acc_y_offset;
        sum_z_offset += (acc_z_offset - ACC_GRABITY);
    }

    // 平均値を算出
    g_acc_x_offset = (sum_x_offset / ACCEL_OFFSET_AVG_CNT) * 10;
    g_acc_y_offset = (sum_y_offset / ACCEL_OFFSET_AVG_CNT) * 10;
    g_acc_z_offset = (sum_z_offset / ACCEL_OFFSET_AVG_CNT) * 10;

    // 表示
    sprite.printf("Accel X Offset: %6.3f\n", g_acc_x_offset);
    sprite.printf("Accel Y Offset: %6.3f\n", g_acc_y_offset);
    sprite.printf("Accel Z Offset: %6.3f\n", g_acc_z_offset);
    sprite.println("Calibrate Finish!");
    sprite.pushSprite(0, 0);

    *p_acc_x_offset = g_acc_x_offset;
    *p_acc_y_offset = g_acc_y_offset;
    *p_acc_z_offset = g_acc_z_offset;
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

    sprite.setCursor(10, 40);
    sprite.printf("Accel X: %6.3f [m/s^2]", g_acc_x);
    sprite.setCursor(10, 70);
    sprite.printf("Accel Y: %6.3f [m/s^2]", g_acc_y);
    sprite.setCursor(10, 100);
    sprite.printf("Accel Z: %6.3f [m/s^2]", g_acc_z);

    sprite.pushSprite(0, 0);
    delay(1000);
}