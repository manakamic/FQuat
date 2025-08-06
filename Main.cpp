//!
//! @file Main.cpp
//!
//! @brief ジンバルロックを確認するサンプル
//!
#include <Windows.h> // WinMain
#include <tchar.h>
#include <functional>
import DxLibWrapper;
import FQuat;

//#define GIMBAL_LOCL
#define EULER
//
// #define QUATERNION
//#define GIMBAL_LOCL_QUATERNION

namespace
{
    constexpr const char* WINDOW_TITLE = _T("FQuat(GimbalLock)");
    constexpr int SCREEN_WIDTH = 1280;
    constexpr int SCREEN_HEIGHT = 720;
    constexpr int SCREEN_DEPTH = 32;
    constexpr const char* MODEL_FILE = _T("character/SDChar.mv1");
    constexpr float TO_RADIAN = DW::DW_PI_F / 180.0f;
    constexpr float DIGREE90 = 90.0f;
    constexpr float MOVING_ANGLE = 1.0f * TO_RADIAN;

    // 並行光源を 1 つ追加する
    const DW::VECTOR LightDir = DW::VGet(1.0f, 0.0f, -0.1f);
    const int  LightHandle = DW::CreateDirLightHandle(LightDir);

    // カメラ情報
    const DW::VECTOR CameraPosition = DW::VGet(100.0f, 100.0f, -300.0f);
    const DW::VECTOR CameraTarget = DW::VGet(0.0f, 0.0f, 0.0f);
    const DW::VECTOR CameraUp = DW::VGet(0.0f, 1.0f, 0.0f);

    // 基準
    const DW::VECTOR BaseX = DW::VGet(1.0f, 0.0f, 0.0f); // X軸
    const DW::VECTOR BaseY = DW::VGet(0.0f, 1.0f, 0.0f); // Y軸
    const DW::VECTOR BaseZ = DW::VGet(0.0f, 0.0f, 1.0f); // Z軸

    // XYZ軸の描画用
    constexpr float LineLength = 100.0f;
    const DW::VECTOR LineOrigin = DW::VGet(0.0f, 0.0f, 0.0f);
    const DW::VECTOR AxisX = DW::VScale(BaseX, LineLength);
    const DW::VECTOR AxisY = DW::VScale(BaseY, LineLength);
    const DW::VECTOR AxisZ = DW::VScale(BaseZ, LineLength);
    const unsigned int ColorX = DW::GetColor(255, 0, 0); // 赤
    const unsigned int ColorY = DW::GetColor(0, 255, 0); // 緑
    const unsigned int ColorZ = DW::GetColor(0, 0, 255); // 青
}

// 前方宣言
std::function<void(void)> CreateProcessEuler(float& AngleX, float& AngleY, float& AngleZ, const DW::MATRIX& RotY, const int& Handle);
#ifndef GIMBAL_LOCL_QUATERNION
std::function<void(void)> CreateProcessQuat(FQuat& RotQuat, const int& Handle);
#else
std::function<void(void)> CreateProcessQuat(float& AngleX, float& AngleY, float& AngleZ, FQuat& RotQuat, const int& Handle);
#endif

int CALLBACK WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nCmdShow)
{
    int window_mode = FALSE;

#ifdef _DEBUG
    window_mode = TRUE;
#endif

    DW::SetMainWindowText(WINDOW_TITLE);
    DW::ChangeWindowMode(window_mode);
    DW::SetGraphMode(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_DEPTH);
    DW::SetBackgroundColor(16, 64, 88);

    if (DW::DxLib_Init() == -1)
    {
        return -1;
    }

    // 3D モデルの読み込み
    const int Handle = DW::MV1LoadModel(MODEL_FILE);

    if (Handle == -1)
    {
        DW::DxLib_End();
        return -1;
    }

    // オイラー角のXYZ軸の計算の真ん中の軸を 90 or -90 度するとジンバルロックが発生する
    float AngleX = 0.0f;
#ifdef GIMBAL_LOCL
    float AngleY = -DIGREE90 * TO_RADIAN;
#else
    float AngleY = 0.0f;
#endif
    float AngleZ = 0.0f;

    DW::MATRIX RotY = DW::MGetRotY(AngleY);
    FQuat RotQuat = FQuat{ BaseY, AngleY };

    std::function<void(void)> ProcessEuler = CreateProcessEuler(AngleX, AngleY, AngleZ, RotY, Handle);
#ifndef GIMBAL_LOCL_QUATERNION
    std::function<void(void)> ProcessQuat = CreateProcessQuat(RotQuat, Handle);
#else
    std::function<void(void)> ProcessQuat = CreateProcessQuat(AngleX, AngleY, AngleZ, RotQuat, Handle);
#endif

    DW::SetDrawScreen(DW::DW_SCREEN_BACK);
    DW::SetCameraPositionAndTargetAndUpVec(CameraPosition, CameraTarget, CameraUp);

    while (DW::ProcessMessage() != -1)
    {
        if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_ESCAPE))
        {
            break;
        }

#ifdef EULER
        ProcessEuler();
#endif
#ifdef QUATERNION
        ProcessQuat();
#endif

        DW::ClearDrawScreen();
        DW::MV1DrawModel(Handle);
        // XYZ軸の描画
        DW::DrawLine3D(LineOrigin, AxisX, ColorX);
        DW::DrawLine3D(LineOrigin, AxisY, ColorY);
        DW::DrawLine3D(LineOrigin, AxisZ, ColorZ);
        DW::ScreenFlip();
    }

    DW::DeleteLightHandle(LightHandle);
    DW::MV1DeleteModel(Handle);
    DW::DxLib_End();

    return 0;
}

void CheckHitKeyAngleXZ(float& AngleX, float& AngleZ)
{
    // 上下キーで X 軸回転
    if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_UP))
    {
        AngleX += MOVING_ANGLE;
    }

    if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_DOWN))
    {
        AngleX -= MOVING_ANGLE;
    }

    // 左右キーで Z 軸回転
    if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_LEFT))
    {
        AngleZ += MOVING_ANGLE;
    }

    if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_RIGHT))
    {
        AngleZ -= MOVING_ANGLE;
    }
}

std::function<void(void)> CreateProcessEuler(float& AngleX, float& AngleY, float& AngleZ, const DW::MATRIX& RotY, const int& Handle)
{
    return [&]()
    {
        CheckHitKeyAngleXZ(AngleX, AngleZ);

        DW::MV1SetRotationXYZ(Handle, DW::VGet(AngleX, AngleY, AngleZ));
        // MV1SetRotationXYZ は下記の処理と同等
        //DW::MATRIX RotX = DW::MGetRotX(AngleX);
        //DW::MATRIX RotZ = DW::MGetRotZ(AngleZ);
        //DW::MATRIX Rot = DW::MMult(DW::MMult(RotX, RotY), RotZ);
        //DW::MV1SetMatrix(Handle, Rot);
    };
}

FQuat CheckHitKeyFQuat(FQuat& RotQuat)
{
    // 上下キーで X 軸回転
    if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_UP))
    {
        RotQuat = FQuat{ BaseX, MOVING_ANGLE } * RotQuat;
    }

    if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_DOWN))
    {
        RotQuat = FQuat{ BaseX, -MOVING_ANGLE } * RotQuat;
    }

    // 左右キーで Z 軸回転
    if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_LEFT))
    {
        RotQuat = FQuat{ BaseZ, MOVING_ANGLE } * RotQuat;
    }

    if (1 == DW::CheckHitKey(DW::DW_KEY_INPUT_RIGHT))
    {
        RotQuat = FQuat{ BaseZ, -MOVING_ANGLE } * RotQuat;
    }

    return RotQuat;
}

#ifndef GIMBAL_LOCL_QUATERNION
std::function<void(void)> CreateProcessQuat(FQuat& RotQuat, const int& Handle)
{
    return [&]() {
        RotQuat = CheckHitKeyFQuat(RotQuat);
        DW::MV1SetMatrix(Handle, RotQuat.ToRotationMatrix());
    };
}
#else
std::function<void(void)> CreateProcessQuat(float& AngleX, float& AngleY, float& AngleZ, FQuat& RotQuat, const int& Handle)
{
    return [&]() {
        CheckHitKeyAngleXZ(AngleX, AngleZ);
        // これは間違った処理
        // オイラー角からクォータニオンを生成してもジンバルロックは回避できない
        // 回転もクォータニオンで行う必要がある
        FQuat Quat = FQuat{ DW::VGet(AngleX, AngleY, AngleZ) };
        DW::MV1SetMatrix(Handle, Quat.ToRotationMatrix());
    };
}
#endif