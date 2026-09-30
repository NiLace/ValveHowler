// GENERATED FILE — DO NOT EDIT BY HAND.
//
// The cascade's fixed filters in `s`, discretised in `prepare()` at
// the session rate, so the cascade runs at any sample rate.
//
// The stage filters are the inverse bilinear transform of the stage SOS
// tables: re-discretising them at the source rate returns those tables
// to machine precision. The rail-current filters (VRA, VRB, VRC, VRP)
// are fitted directly in `s` and copied as they are.
//
// Convention: `s` in units of kScale = 2*pi*1000 (same as tone and
// level), coefficients from highest to lowest power, monic denominator.
//
// Generator invocation:
//     --stage1 src/nls_stage1_coef_v9ri_384k.h
//     --e234   src/nls_e234_coef_v9ri_384k.h
//     --rail   src/nls_rail_n3_v9ri_384k.h
//     --railvr src/nls_rail_vr_v9ri.h
//     --output src/nls_fijos_s_v9ri.h
#pragma once

namespace nlsc {
namespace fixed_v9ri {

static constexpr double kScale = 6283.1853071795858;

// stage1::H1 — stage 1 · input network
static constexpr int kH1_NB = 4;
static constexpr int kH1_NA = 4;
static constexpr double kH1_B[5][1] = {
    {+1.44842613305690864e-03},
    {+2.56200147049185887e+02},
    {+3.68722450322739996e+03},
    {+5.82011518966039745e+01},
    {+4.59788450140497149e-07},
};
static constexpr double kH1_A[5][1] = {
    {+6.10869577234005902e-03},
    {+2.56905327965458866e+02},
    {+3.70119213883103066e+03},
    {+1.27167758510492249e+02},
    {+1.00000000000000000e+00},
};

// stage1::HS — stage 1 · waveshaper network
static constexpr int kHS_NB = 6;
static constexpr int kHS_NA = 6;
static constexpr double kHS_B[7][1] = {
    {+1.68021702992819545e-08},
    {+6.96017826034328369e-05},
    {+1.48054926790867764e-01},
    {+1.78277336110952334e+01},
    {+9.09838756497475885e+01},
    {+6.47285040921000387e+01},
    {+1.00723102954479593e+00},
};
static constexpr double kHS_A[7][1] = {
    {+1.68500036957858240e-08},
    {+6.97061332802304391e-05},
    {+1.48066601403675779e-01},
    {+1.78276667413454923e+01},
    {+9.09810757241272086e+01},
    {+6.47175740639180646e+01},
    {+1.00000000000000000e+00},
};

// stage1::H2 — stage 1 · output network
static constexpr int kH2_NB = 4;
static constexpr int kH2_NA = 4;
static constexpr double kH2_B[5][1] = {
    {+1.37467613531166535e+02},
    {+7.44612008725471242e+02},
    {+5.96901238189231208e+02},
    {+6.28306461846555493e+01},
    {+1.99912054641546314e-06},
};
static constexpr double kH2_A[5][1] = {
    {+1.37467614541727528e+02},
    {+7.46800730945949340e+02},
    {+6.08752498580710721e+02},
    {+7.23308025061455311e+01},
    {+1.00000000000000000e+00},
};

// e234::E4GS — stage 4 · charge step n17/n19
static constexpr int kE4GS_NB = 6;
static constexpr int kE4GS_NA = 6;
static constexpr double kE4GS_B[7][1] = {
    {-6.51053915821686138e-12},
    {+4.73917320358242249e-05},
    {+7.65138965731209601e-03},
    {+2.46639448201333350e-01},
    {+2.40383732432411845e+00},
    {+2.99295064172677705e+00},
    {+9.99997488729825190e-01},
};
static constexpr double kE4GS_A[7][1] = {
    {+2.51531027565088979e-11},
    {+4.73967409810028656e-05},
    {+7.65153811913385669e-03},
    {+2.46640593838294037e-01},
    {+2.40383869678418094e+00},
    {+2.99295175917326750e+00},
    {+1.00000000000000000e+00},
};

// e234::E4G2 — stage 4 · output out/n17
static constexpr int kE4G2_NB = 2;
static constexpr int kE4G2_NA = 2;
static constexpr double kE4G2_B[3][1] = {
    {+5.14038856455475113e+01},
    {+6.28314815591593197e+03},
    {+5.89455593674073253e-06},
};
static constexpr double kE4G2_A[3][1] = {
    {+5.16454839063046620e+01},
    {+6.31268713321395899e+03},
    {+1.00000000000000000e+00},
};

// e234::E2C — opamp correction (off)
static constexpr int kE2C_NB = 2;
static constexpr int kE2C_NA = 2;
static constexpr double kE2C_B[3][1] = {
    {+6.69325385273529489e-05},
    {+1.63624617374468380e-02},
    {+1.00000000000000000e+00},
};
static constexpr double kE2C_A[3][1] = {
    {+6.69325385273529489e-05},
    {+1.63624617374468380e-02},
    {+1.00000000000000000e+00},
};

// rail::N3 — VR -> n3 (stage 1, via R2 510k)
static constexpr int kN3_NB = 6;
static constexpr int kN3_NA = 6;
static constexpr double kN3_B[7][1] = {
    {+1.20994675463115512e-05},
    {+1.97706284184813348e-02},
    {+2.47704343369563551e+00},
    {+3.03194580617467153e+01},
    {+1.02414647012204711e+02},
    {+6.44622520174713145e+01},
    {+9.08136811342536765e-01},
};
static constexpr double kN3_A[7][1] = {
    {+2.85589446757108243e-04},
    {+8.49355699444772938e+00},
    {+1.07904225799406186e+03},
    {+5.54842401668909315e+03},
    {+3.88239423233922753e+03},
    {+1.28591731133433171e+02},
    {+1.00000000000000000e+00},
};

// e234::VRA — current n7 feeds the rail
static constexpr int kVRA_NB = 2;
static constexpr int kVRA_NA = 2;
static constexpr double kVRA_B[3][1] = {
    {+1.40201907062354203e-14},
    {+7.23431115802924859e-05},
    {+1.00259353654030092e-05},
};
static constexpr double kVRA_A[3][1] = {
    {+1.00000000000000000e+00},
    {+1.00366027862752816e+00},
    {+1.10285539862215370e-01},
};

// e234::VRB — current n14 feeds the rail
static constexpr int kVRB_NB = 2;
static constexpr int kVRB_NA = 2;
static constexpr double kVRB_B[3][1] = {
    {+3.82743810778096984e-16},
    {+1.59154782267731496e-05},
    {-5.88409990068610033e-19},
};
static constexpr double kVRB_A[3][1] = {
    {+1.00000000000000000e+00},
    {+1.76661811639499788e+00},
    {+2.53302701483997233e-03},
};

// e234::VRC — current n19 feeds the rail
static constexpr int kVRC_NB = 2;
static constexpr int kVRC_NA = 2;
static constexpr double kVRC_B[3][1] = {
    {+1.19607842106419815e-05},
    {+5.05550491739169013e-06},
    {+4.96671964602191047e-09},
};
static constexpr double kVRC_A[3][1] = {
    {+1.00000000000000000e+00},
    {+1.76661811762567922e+00},
    {+2.53302701881972245e-03},
};

// e234::VRP — node VR impedance
static constexpr int kVRP_NB = 2;
static constexpr int kVRP_NA = 2;
static constexpr double kVRP_B[3][1] = {
    {+2.61410331517325587e-04},
    {+3.38608032546110937e+00},
    {+4.85925663148351061e-03},
};
static constexpr double kVRP_A[3][1] = {
    {+1.00000000000000000e+00},
    {+3.52602456743456418e-03},
    {+2.95246803072482935e-06},
};

} // namespace fixed_v9ri
} // namespace nlsc
