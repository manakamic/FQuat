export module FQuat;
import DxLibWrapper;

// 定数
inline constexpr float QUAT_SMALL_NUMBER = 1.e-8f;

/**
 * @brief Unreal Engine の FQuat と同様の機能を持つクォータニオンを実装します。
 * DXライブラリの座標系 (Y-up) に合わせてあります。
 */
export struct FQuat {
public:
    // static メソッド
    static constexpr float Dot(const FQuat& Q1, const FQuat& Q2);
    static FQuat Slerp(const FQuat& Quat1, const FQuat& Quat2, float Alpha);

public:
    constexpr FQuat() noexcept = default;
    constexpr FQuat(float InX, float InY, float InZ, float InW) noexcept
        : X(InX), Y(InY), Z(InZ), W(InW) {
    }
    FQuat(const DW::VECTOR& Axis, float AngleRad) noexcept;
    explicit FQuat(const DW::VECTOR& Euler) noexcept;

    constexpr FQuat(const FQuat& Other) noexcept = default;
    constexpr FQuat(FQuat&& Other) noexcept = default;
    FQuat& operator =(const FQuat& Other) noexcept = default;
    FQuat& operator =(FQuat&& Other) noexcept = default;

    FQuat operator *(const FQuat& Q) const noexcept;
    FQuat& operator *=(const FQuat& Q) noexcept;
    DW::VECTOR operator *(const DW::VECTOR& V) const noexcept;

    void Normalize(const float Tolerance = QUAT_SMALL_NUMBER) noexcept;
    [[nodiscard]] FQuat GetNormalized(const float Tolerance = QUAT_SMALL_NUMBER) const noexcept;
    [[nodiscard]] constexpr FQuat Inverse() const noexcept;
    [[nodiscard]] float Size() const noexcept;
    [[nodiscard]] constexpr float SizeSquared() const noexcept;
    [[nodiscard]] DW::MATRIX ToRotationMatrix() const noexcept;
    [[nodiscard]] DW::VECTOR ToEuler() const noexcept;
    [[nodiscard]] DW::VECTOR RotateVector(const DW::VECTOR& V) const noexcept;
    [[nodiscard]] DW::VECTOR UnrotateVector(const DW::VECTOR& V) const noexcept;

public:
    float X{ 0.0f };
    float Y{ 0.0f };
    float Z{ 0.0f };
    float W{ 1.0f };
};

/**
 * @brief 単位クォータニオン (X=0, Y=0, Z=0, W=1). 回転を行わないクォータニオン
 */
export inline constexpr FQuat FQuatIdentity = FQuat{ 0.0f, 0.0f, 0.0f, 1.0f };
