#include <M5Core2.h>
#include <stdint.h>

#define ACCEL_AVG_NUM    100

// LCDのスプライト
TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);

float g_acc_x_f = 0.0;
float g_acc_y_f = 0.0;
float g_acc_z_f = 0.0;

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
        sum_x += acc_x;
        sum_y += acc_y;
        sum_z += acc_z;
    }

    // 平均値を算出
    g_acc_x_f = (sum_x / ACCEL_AVG_NUM) * 10;
    g_acc_y_f = (sum_y / ACCEL_AVG_NUM) * 10;
    g_acc_z_f = (sum_z / ACCEL_AVG_NUM) * 10;

    // 表示
    sprite.fillSprite(TFT_BLACK);
    sprite.setCursor(0, 0);
    sprite.println("3-Axis Accel Develop");

    sprite.setCursor(10, 40);
    sprite.printf("Accel X: %6.3f [m/s^2]", g_acc_x_f);
    sprite.setCursor(10, 70);
    sprite.printf("Accel Y: %6.3f [m/s^2]", g_acc_y_f);
    sprite.setCursor(10, 100);
    sprite.printf("Accel Z: %6.3f [m/s^2]", g_acc_z_f);

    sprite.pushSprite(0, 0);
    delay(1000);
}