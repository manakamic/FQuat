#include <cmath>
module FQuat;
import DxLibWrapper;

/**
 * @brief 回転軸(正規化不要)と回転角度(ラジアン)からクォータニオンを生成します.
 * @param Axis 回転軸ベクトル.
 * @param AngleRad 回転角度 (ラジアン).
 */
FQuat::FQuat(const DW::VECTOR& Axis, float AngleRad) noexcept
{
    const float HalfAngle = AngleRad * 0.5f;
    const float Sin = std::sin(HalfAngle);
    const float Cos = std::cos(HalfAngle);
    const DW::VECTOR NormAxis = DW::VNorm(Axis);

    X = NormAxis.x * Sin;
    Y = NormAxis.y * Sin;
    Z = NormAxis.z * Sin;
    W = Cos;
}

/**
 * @brief オイラー角(ラジアン)からクォータニオンを生成します.
 * 回転順序は Roll(X軸回転), Pitch(Y軸回転), Yaw(Z軸回転) です.
 * @param Euler オイラー角 (X:Roll, Y:Pitch, Z:Yaw).
 */
FQuat::FQuat(const DW::VECTOR& Euler) noexcept
{
    const float HalfRoll = Euler.x * 0.5f;
    const float HalfPitch = Euler.y * 0.5f;
    const float HalfYaw = Euler.z * 0.5f;
    const float SR = std::sin(HalfRoll);
    const float CR = std::cos(HalfRoll);
    const float SP = std::sin(HalfPitch);
    const float CP = std::cos(HalfPitch);
    const float SY = std::sin(HalfYaw);
    const float CY = std::cos(HalfYaw);

    W = CR * CP * CY + SR * SP * SY;
    X = SR * CP * CY - CR * SP * SY;
    Y = CR * SP * CY + SR * CP * SY;
    Z = CR * CP * SY - SR * SP * CY;
}

/**
 * @brief クォータニオン同士の積. 回転を合成します.
 */
FQuat FQuat::operator *(const FQuat& Q) const noexcept
{
    return FQuat{
        W * Q.X + X * Q.W + Y * Q.Z - Z * Q.Y,
        W * Q.Y - X * Q.Z + Y * Q.W + Z * Q.X,
        W * Q.Z + X * Q.Y - Y * Q.X + Z * Q.W,
        W * Q.W - X * Q.X - Y * Q.Y - Z * Q.Z };
}

FQuat& FQuat::operator *=(const FQuat& Q) noexcept
{
    *this = *this * Q;
    return *this;
}

/**
 * @brief ベクトルをこのクォータニオンで回転させます.
 */
DW::VECTOR FQuat::operator *(const DW::VECTOR& V) const noexcept
{
    return RotateVector(V);
}

/**
 * @brief このクォータニオンを正規化します.
 */
void FQuat::Normalize(const float Tolerance) noexcept
{
    const float MagSq = SizeSquared();

    if (MagSq > Tolerance)
    {
        const float InvMag = 1.0f / std::sqrt(MagSq);

        X *= InvMag;
        Y *= InvMag;
        Z *= InvMag;
        W *= InvMag;
    }
    else
    {
        *this = FQuatIdentity;
    }
}

/**
 * @brief 正規化されたこのクォータニオンのコピーを返します.
 */
FQuat FQuat::GetNormalized(const float Tolerance) const noexcept
{
    FQuat Result = *this;
    Result.Normalize(Tolerance);

    return Result;
}

/**
 * @brief 逆クォータニオンを返します. 正規化されているクォータニオンの場合、これは逆回転を表します.
 */
constexpr FQuat FQuat::Inverse() const noexcept
{
    return FQuat{ -X, -Y, -Z, W };
}

/**
 * @brief クォータニオンの長さ(大きさ)を返します.
 */
float FQuat::Size() const noexcept
{
    return std::sqrt(SizeSquared());
}

/**
 * @brief クォータニオンの長さの2乗を返します. 長さの比較などに使用します.
 */
constexpr float FQuat::SizeSquared() const noexcept
{
    return (X * X) + (Y * Y) + (Z * Z) + (W * W);
}

/**
 * @brief このクォータニオンが表す回転をDXライブラリの回転行列に変換します.
 */
DW::MATRIX FQuat::ToRotationMatrix() const noexcept
{
    DW::MATRIX M = DW::MGetIdent();
    const float xx = X * X, yy = Y * Y, zz = Z * Z;
    const float xy = X * Y, xz = X * Z, yz = Y * Z;
    const float xw = X * W, yw = Y * W, zw = Z * W;

    M.m[0][0] = 1.0f - 2.0f * (yy + zz);
    M.m[0][1] = 2.0f * (xy + zw);
    M.m[0][2] = 2.0f * (xz - yw);

    M.m[1][0] = 2.0f * (xy - zw);
    M.m[1][1] = 1.0f - 2.0f * (xx + zz);
    M.m[1][2] = 2.0f * (yz + xw);

    M.m[2][0] = 2.0f * (xz + yw);
    M.m[2][1] = 2.0f * (yz - xw);
    M.m[2][2] = 1.0f - 2.0f * (xx + yy);

    return M;
}

/**
 * @brief このクォータニオンをオイラー角(ラジアン ZXY オーダー)に変換します.
 * @return VECTOR (X:Roll, Y:Pitch, Z:Yaw).
 */
DW::VECTOR FQuat::ToEuler() const noexcept
{
    DW::VECTOR Euler; // Roll, Pitch, Yaw

    // Roll (X軸回転)
    const float SinR_CosP = 2.0f * (W * X + Y * Z);
    const float CosR_CosP = 1.0f - 2.0f * (X * X + Y * Y);

    Euler.x = atan2f(SinR_CosP, CosR_CosP);

    // Pitch (Y軸回転)
    const float SinP = 2.0f * (W * Y - Z * X);

    if (fabsf(SinP) >= 1.0f)
    {
        Euler.y = copysignf(DW::DW_PI_F / 2.0f, SinP); // ジンバルロック対策
    }
    else
    {
        Euler.y = asinf(SinP);
    }

    // Yaw (Z軸回転)
    const float SinY_CosP = 2.0f * (W * Z + X * Y);
    const float CosY_CosP = 1.0f - 2.0f * (Y * Y + Z * Z);

    Euler.z = atan2f(SinY_CosP, CosY_CosP);

    return Euler;
}

/**
 * @brief ベクトルをこのクォータニオンで回転させます.
 */
DW::VECTOR FQuat::RotateVector(const DW::VECTOR& V) const noexcept
{
    const DW::VECTOR QuatVector = DW::VGet(X, Y, Z);
    const DW::VECTOR T = DW::VScale(DW::VCross(QuatVector, V), 2.0f);

    return DW::VAdd(V, DW::VAdd(DW::VScale(T, W), DW::VCross(QuatVector, T)));
}

/**
 * @brief ベクトルをこのクォータニオンの逆回転で回転させます.
 */
DW::VECTOR FQuat::UnrotateVector(const DW::VECTOR& V) const noexcept
{
    return this->Inverse().RotateVector(V);
}

/**
 * @brief 2つのクォータニオンの内積を計算します.
 */
constexpr float FQuat::Dot(const FQuat& Q1, const FQuat& Q2)
{
    return Q1.X * Q2.X + Q1.Y * Q2.Y + Q1.Z * Q2.Z + Q1.W * Q2.W;
}

/**
 * @brief 2つのクォータニオン間を球面線形補間(Slerp)します.
 * @param Quat1 開始クォータニオン (正規化済みであること).
 * @param Quat2 終了クォータニオン (正規化済みであること).
 * @param Alpha 補間係数 (0.0 - 1.0).
 * @return 補間されたクォータニオン.
 */
FQuat FQuat::Slerp(const FQuat& Quat1, const FQuat& Quat2, float Alpha)
{
    const float RawCosom = Dot(Quat1, Quat2);
    float Cosom = RawCosom;
    FQuat EndQuat = Quat2;

    if (Cosom < 0.0f)
    {
        EndQuat = FQuat{ -Quat2.X, -Quat2.Y, -Quat2.Z, -Quat2.W };
        Cosom = -Cosom;
    }

    float Scale0, Scale1;

    if (Cosom < 0.9999f)
    {
        const float Omega = std::acos(Cosom);
        const float InvSin = 1.0f / std::sin(Omega);

        Scale0 = std::sin((1.0f - Alpha) * Omega) * InvSin;
        Scale1 = std::sin(Alpha * Omega) * InvSin;
    }
    else
    {
        // 角度が非常に近い場合は線形補間
        Scale0 = 1.0f - Alpha;
        Scale1 = Alpha;
    }

    return FQuat{
        Scale0 * Quat1.X + Scale1 * EndQuat.X,
        Scale0 * Quat1.Y + Scale1 * EndQuat.Y,
        Scale0 * Quat1.Z + Scale1 * EndQuat.Z,
        Scale0 * Quat1.W + Scale1 * EndQuat.W
    }.GetNormalized();
}