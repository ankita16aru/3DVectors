module BitMultiplier8(
    input [7:0] a,b,
    output [15:0] p
);

wire [128:0] w;


and a1(w[0], a[0], b[0]);
and a2(w[1], a[1], b[0]);
and a3(w[2], a[2], b[0]);
and a4(w[3], a[3], b[0]);
and a5(w[4], a[4], b[0]);
and a6(w[5], a[5], b[0]);
and a7(w[6], a[6], b[0]);
and a8(w[7], a[7], b[0]);

and a9(w[8], a[0], b[1]);
and a10(w[9], a[1], b[1]);
and a11(w[10], a[2], b[1]);
and a12(w[11], a[3], b[1]);
and a13(w[12], a[4], b[1]);
and a14(w[13], a[5], b[1]);
and a15(w[14], a[6], b[1]);
and a16(w[15], a[7], b[1]);

and a17(w[16], a[0], b[2]);
and a18(w[17], a[1], b[2]);
and a19(w[18], a[2], b[2]);
and a20(w[19], a[3], b[2]);
and a21(w[20], a[4], b[2]);
and a22(w[21], a[5], b[2]);
and a23(w[22], a[6], b[2]);
and a24(w[23], a[7], b[2]);

and a25(w[24], a[0], b[3]);
and a26(w[25], a[1], b[3]);
and a27(w[26], a[2], b[3]);
and a28(w[27], a[3], b[3]);
and a29(w[28], a[4], b[3]);
and a30(w[29], a[5], b[3]);
and a31(w[30], a[6], b[3]);
and a32(w[31], a[7], b[3]);

and a33(w[32], a[0], b[4]);
and a34(w[33], a[1], b[4]);
and a35(w[34], a[2], b[4]);
and a36(w[35], a[3], b[4]);
and a37(w[36], a[4], b[4]);
and a38(w[37], a[5], b[4]);
and a39(w[38], a[6], b[4]);
and a40(w[39], a[7], b[4]);

and a41(w[40], a[0], b[5]);
and a42(w[41], a[1], b[5]);
and a43(w[42], a[2], b[5]);
and a44(w[43], a[3], b[5]);
and a45(w[44], a[4], b[5]);
and a46(w[45], a[5], b[5]);
and a47(w[46], a[6], b[5]);
and a48(w[47], a[7], b[5]);

and a49(w[48], a[0], b[6]);
and a50(w[49], a[1], b[6]);
and a51(w[50], a[2], b[6]);
and a52(w[51], a[3], b[6]);
and a53(w[52], a[4], b[6]);
and a54(w[53], a[5], b[6]);
and a55(w[54], a[6], b[6]);
and a56(w[55], a[7], b[6]);

and a57(w[56], a[0], b[7]);
and a58(w[57], a[1], b[7]);
and a59(w[58], a[2], b[7]);
and a60(w[59], a[3], b[7]);
and a61(w[60], a[4], b[7]);
and a62(w[61], a[5], b[7]);
and a63(w[62], a[6], b[7]);
and a64(w[63], a[7], b[7]);

fulladder fa0(w[0], w[8], 1'b0, p[0], w[64]);
fulladder fa1(w[1], w[9], w[64], p[1], w[65]);
fulladder fa2(w[2], w[10], w[65], p[2], w[66]);
fulladder fa3(w[3], w[11], w[66], p[3], w[67]);
fulladder fa4(w[4], w[12], w[67], p[4], w[68]);
fulladder fa5(w[5], w[13], w[68], p[5], w[69]);
fulladder fa6(w[6], w[14], w[69], p[6], w[70]);
fulladder fa7(w[7], w[15], w[70], p[7], w[71]);


assign p[8] = w[72];
assign p[9] = w[80];
assign p[10] = w[88];
assign p[11] = w[96];
assign p[12] = w[104];
assign p[13] = w[112];
assign p[14] = w[120];
assign p[15] = w[128];

endmodule

module fulladder(
    input a, b, cin,
    output sum, cout
);

    wire x1, x2, x3;

    xor(x1, a, b);
    and(x3, a, b);
    xor(sum, x1, cin);
    and(x2, x1, cin);
    or(cout, x2, x3);

endmodule

module halfadder(
    input a, b,
    output sum, out
);

    xor(sum, a, b);
    and(out, a, b);

endmodule

module RippleCarryAdder(
    input [7:0] a,
    input [7:0] b,
    output [15:0] sum
);

    wire [15:0] carry;

    fulladder fa0(a[0], b[0], 1'b0, sum[0], carry[0]);
    fulladder fa1(a[1], b[1], carry[0], sum[1], carry[1]);
    fulladder fa2(a[2], b[2], carry[1], sum[2], carry[2]);
    fulladder fa3(a[3], b[3], carry[2], sum[3], carry[3]);
    fulladder fa4(a[4], b[4], carry[3], sum[4], carry[4]);
    fulladder fa5(a[5], b[5], carry[4], sum[5], carry[5]);
    fulladder fa6(a[6], b[6], carry[5], sum[6], carry[6]);
    fulladder fa7(a[7], b[7], carry[6], sum[7], carry[7]);

    fulladder fa00(1'b0, 1'b0, carry[7], sum[8], carry[8]);
    fulladder fa01(1'b0, 1'b0, 1'b0, sum[9], carry[9]);
    fulladder fa02(1'b0, 1'b0, 1'b0, sum[10], carry[10]);
    fulladder fa03(1'b0, 1'b0, 1'b0, sum[11], carry[11]);
    fulladder fa04(1'b0, 1'b0, 1'b0, sum[12], carry[12]);
    fulladder fa05(1'b0, 1'b0, 1'b0, sum[13], carry[13]);
    fulladder fa06(1'b0, 1'b0, 1'b0, sum[14], carry[14]);
    fulladder fa07(1'b0, 1'b0, 1'b0, sum[15], carry[15]);

endmodule

module RippleCarrySubtractor(
    input [7:0] a,
    input [7:0] b,
    output [15:0] sum
);

    wire [15:0] carry;

    fulladder fa0(a[0], ~b[0], 1'b1, sum[0], carry[0]);
    fulladder fa1(a[1], ~b[1], carry[0], sum[1], carry[1]);
    fulladder fa2(a[2], ~b[2], carry[1], sum[2], carry[2]);
    fulladder fa3(a[3], ~b[3], carry[2], sum[3], carry[3]);
    fulladder fa4(a[4], ~b[4], carry[3], sum[4], carry[4]);
    fulladder fa5(a[5], ~b[5], carry[4], sum[5], carry[5]);
    fulladder fa6(a[6], ~b[6], carry[5], sum[6], carry[6]);
    fulladder fa7(a[7], ~b[7], carry[6], sum[7], carry[7]);

    fulladder fa00(1'b0, 1'b0, carry[7], sum[8], carry[8]);
    fulladder fa01(1'b0, 1'b0, 1'b0, sum[9], carry[9]);
    fulladder fa02(1'b0, 1'b0, 1'b0, sum[10], carry[10]);
    fulladder fa03(1'b0, 1'b0, 1'b0, sum[11], carry[11]);
    fulladder fa04(1'b0, 1'b0, 1'b0, sum[12], carry[12]);
    fulladder fa05(1'b0, 1'b0, 1'b0, sum[13], carry[13]);
    fulladder fa06(1'b0, 1'b0, 1'b0, sum[14], carry[14]);
    fulladder fa07(1'b0, 1'b0, 1'b0, sum[15], carry[15]);

endmodule

module Adder16bit(
    input [15:0] a,
    input [15:0] b,
    output [15:0] sum
);

    wire [15:0] carry;

    fulladder fa0(a[0], b[0], 1'b0, sum[0], carry[0]);
    fulladder fa1(a[1], b[1], carry[0], sum[1], carry[1]);
    fulladder fa2(a[2], b[2], carry[1], sum[2], carry[2]);
    fulladder fa3(a[3], b[3], carry[2], sum[3], carry[3]);
    fulladder fa4(a[4], b[4], carry[3], sum[4], carry[4]);
    fulladder fa5(a[5], b[5], carry[4], sum[5], carry[5]);
    fulladder fa6(a[6], b[6], carry[5], sum[6], carry[6]);
    fulladder fa7(a[7], b[7], carry[6], sum[7], carry[7]);
    fulladder fa8(a[8], b[8], carry[7], sum[8], carry[8]);
    fulladder fa9(a[9], b[9], carry[8], sum[9], carry[9]);
    fulladder fa10(a[10], b[10], carry[9], sum[10], carry[10]);
    fulladder fa11(a[11], b[11], carry[10], sum[11], carry[11]);
    fulladder fa12(a[12], b[12], carry[11], sum[12], carry[12]);
    fulladder fa13(a[13], b[13], carry[12], sum[13], carry[13]);
    fulladder fa14(a[14], b[14], carry[13], sum[14], carry[14]);
    fulladder fa15(a[15], b[15], carry[14], sum[15], carry[15]);

endmodule

module Sub16bit(
    input [15:0] a,
    input [15:0] b,
    output [15:0] sum
);

    wire [15:0] carry;

    fulladder fa0(a[0], ~b[0], 1'b1, sum[0], carry[0]);
    fulladder fa1(a[1], ~b[1], carry[0], sum[1], carry[1]);
    fulladder fa2(a[2], ~b[2], carry[1], sum[2], carry[2]);
    fulladder fa3(a[3], ~b[3], carry[2], sum[3], carry[3]);
    fulladder fa4(a[4], ~b[4], carry[3], sum[4], carry[4]);
    fulladder fa5(a[5], ~b[5], carry[4], sum[5], carry[5]);
    fulladder fa6(a[6], ~b[6], carry[5], sum[6], carry[6]);
    fulladder fa7(a[7], ~b[7], carry[6], sum[7], carry[7]);
    fulladder fa8(a[8], ~b[8], carry[7], sum[8], carry[8]);
    fulladder fa9(a[9], ~b[9], carry[8], sum[9], carry[9]);
    fulladder fa10(a[10], ~b[10], carry[9], sum[10], carry[10]);
    fulladder fa11(a[11], ~b[11], carry[10], sum[11], carry[11]);
    fulladder fa12(a[12], ~b[12], carry[11], sum[12], carry[12]);
    fulladder fa13(a[13], ~b[13], carry[12], sum[13], carry[13]);
    fulladder fa14(a[14], ~b[14], carry[13], sum[14], carry[14]);
    fulladder fa15(a[15], ~b[15], carry[14], sum[15], carry[15]);

    endmodule

module CROSS(
    input [7:0] ax,ay,az,bx,by,bz,
    output [15:0] rx,ry,rz
);

    wire [15:0] exp1, exp2, exp3, exp4, exp5, exp6;

    BitMultiplier8 bm1(ay, bz, exp1);
    BitMultiplier8 bm2(az, by, exp2);
    BitMultiplier8 bm3(az, bx, exp3);
    BitMultiplier8 bm4(ax, bz, exp4);
    BitMultiplier8 bm5(ax, by, exp5);
    BitMultiplier8 bm6(ay, bx, exp6);

    wire [15:0] crossx, crossy, crossz;

    Sub16bit sub1(exp1, exp2, rx);
    Sub16bit sub2(exp3, exp4, ry);
    Sub16bit sub3(exp5, exp6, rz);

endmodule

module ScalaraMult(
    input [7:0] ax,ay,az,
    input [7:0] scalar
    output [15:0] rx,ry,rz
);

    BitMultiplier8 bm1(ax, scalar, rx);
    BitMultiplier8 bm2(ay, scalar, ry);
    BitMultiplier8 bm3(az, scalar, rz);

endmodule

module DotProduct(
    input [7:0] ax,ay,az,bx,by,bz,
    output [15:0] ans
);

    wire [15:0] exp1, exp2, exp3;

    BitMultiplier8 bm1(ax, bx, exp1);
    BitMultiplier8 bm2(ay, by, exp2);
    BitMultiplier8 bm3(az, bz, exp3);

    wire [15:0] ans_buffer;

    Adder16bit add1(exp1, exp2, ans_buffer);
    Adder16bit add2(ans, exp3, ans);

endmodule

module Comparator(
    input [7:0] ax,ay,az,bx,by,bz,
    output [1:0] ans
);

endmodule



module ALU3D(
    input [7:0] ax,ay,az,bx,by,bz,
    input [2:0] op,
    output [15:0] rx,ry,rz
    output [15:0] ans
);

    parameter ADD = 0;
    parameter SUB = 1;
    parameter CROSS = 2;
    parameter SCALAR = 3;
    parameter DOT = 4;
    parameter COMPARATOR = 5;
    parameter ORTHOGONAL = 6;

    wire [15:0] sumx, sumy, sumz;
    wire [15:0] subx, suby, subz;
    wire [15:0] scalarx, scalary, scalarz;
    wire [15:0] dot;
    wire [15:0] crossx, crossy, crossz;
    wire [1:0] comparator;
    wire orthogonal;

    // ALU

    RippleCarryAdder adderX(ax, bx, sumx);
    RippleCarryAdder adderY(ay, by, sumy);
    RippleCarryAdder adderZ(az, bz, sumz);

    RippleCarrySubtractor subX(ax, bx, subx);
    RippleCarrySubtractor subY(ay, by, suby);
    RippleCarrySubtractor subZ(az, bz, subz);

    ScalaraMult scalar(ax, ay, az, scalarx, scalary, scalarz);

    DotProduct dot(ax, ay, az, bx, by, bz, dot);

    CROSS cross(ax, ay, az, bx, by, bz, crossx, crossy, crossz);

    Comparator comp(ax, ay, az, bx, by, bz, comparator);

    Orthogonal orth(ax, ay, az, bx, by, bz, orthogonal);

    // MUX

    
    
endmodule