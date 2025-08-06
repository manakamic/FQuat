#include <DxLib.h>
export module DxLibWrapper;

export namespace DW {
    using ::SetGraphMode;
    using ::SetMainWindowText;
    using ::ChangeWindowMode;
    using ::SetBackgroundColor;
    using ::DxLib_Init;
    using ::SetDrawScreen;
    using ::ProcessMessage;
    using ::ClearDrawScreen;
    using ::ScreenFlip;
    using ::DxLib_End;

    using ::CheckHitKey;
    using ::GetColor;

    using ::CreateDirLightHandle;
    using ::DeleteLightHandle;

    using ::VGet;
    using ::VAdd;
    using ::VNorm;
    using ::VScale;
    using ::VCross;
    using ::MGetIdent;
    using ::MGetRotX;
    using ::MGetRotY;
    using ::MGetRotZ;
    using ::MMult;
    using ::MV1SetRotationXYZ;
    using ::MV1SetMatrix;
    using ::SetCameraPositionAndTargetAndUpVec;

    using ::MV1LoadModel;
    using ::MV1DrawModel;
    using ::MV1DeleteModel;
    using ::DrawLine3D;

    using ::VECTOR;
    using ::MATRIX;

    export constexpr auto DW_PI_F = DX_PI_F;
    export constexpr auto DW_SCREEN_BACK = DX_SCREEN_BACK;
    export constexpr auto DW_KEY_INPUT_ESCAPE = KEY_INPUT_ESCAPE;
    export constexpr auto DW_KEY_INPUT_SPACE = KEY_INPUT_SPACE;
    export constexpr auto DW_KEY_INPUT_LEFT = KEY_INPUT_LEFT;
    export constexpr auto DW_KEY_INPUT_UP = KEY_INPUT_UP;
    export constexpr auto DW_KEY_INPUT_RIGHT = KEY_INPUT_RIGHT;
    export constexpr auto DW_KEY_INPUT_DOWN = KEY_INPUT_DOWN;
}
