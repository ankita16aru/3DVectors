
module Bitmultiplier8(
    input [7:0] a,b,
    output [15:0] p
);

wire [62:0]w;
wire [41:0]s;
wire [54:0]c;


and a1(p[0], a[0], b[0]);
and a2(w[0], a[1], b[0]);
and a3(w[1], a[2], b[0]);
and a4(w[2], a[3], b[0]);
and a5(w[3], a[4], b[0]);
and a6(w[4], a[5], b[0]);
and a7(w[5], a[6], b[0]);
and a8(w[6], a[7], b[0]);

and a9(w[7], a[0], b[1]);
and a10(w[8], a[1], b[1]);
and a11(w[9], a[2], b[1]);
and a12(w[10], a[3], b[1]);
and a13(w[11], a[4], b[1]);
and a14(w[12], a[5], b[1]);
and a15(w[13], a[6], b[1]);
and a16(w[14], a[7], b[1]);

and a17(w[15], a[0], b[2]);
and a18(w[16], a[1], b[2]);
and a19(w[17], a[2], b[2]);
and a20(w[18], a[3], b[2]);
and a21(w[19], a[4], b[2]);
and a22(w[20], a[5], b[2]);
and a23(w[21], a[6], b[2]);
and a24(w[22], a[7], b[2]);

and a25(w[23], a[0], b[3]);
and a26(w[24], a[1], b[3]);
and a27(w[25], a[2], b[3]);
and a28(w[26], a[3], b[3]);
and a29(w[27], a[4], b[3]);
and a30(w[28], a[5], b[3]);
and a31(w[29], a[6], b[3]);
and a32(w[30], a[7], b[3]);

and a33(w[31], a[0], b[4]);
and a34(w[32], a[1], b[4]);
and a35(w[33], a[2], b[4]);
and a36(w[34], a[3], b[4]);
and a37(w[35], a[4], b[4]);
and a38(w[36], a[5], b[4]);
and a39(w[37], a[6], b[4]);
and a40(w[38], a[7], b[4]);

and a41(w[39], a[0], b[5]);
and a42(w[40], a[1], b[5]);
and a43(w[41], a[2], b[5]);
and a44(w[42], a[3], b[5]);
and a45(w[43], a[4], b[5]);
and a46(w[44], a[5], b[5]);
and a47(w[45], a[6], b[5]);
and a48(w[46], a[7], b[5]);

and a49(w[47], a[0], b[6]);
and a50(w[48], a[1], b[6]);
and a51(w[49], a[2], b[6]);
and a52(w[50], a[3], b[6]);
and a53(w[51], a[4], b[6]);
and a54(w[52], a[5], b[6]);
and a55(w[53], a[6], b[6]);
and a56(w[54], a[7], b[6]);

and a57(w[55], a[0], b[7]);
and a58(w[56], a[1], b[7]);
and a59(w[57], a[2], b[7]);
and a60(w[58], a[3], b[7]);
and a61(w[59], a[4], b[7]);
and a62(w[60], a[5], b[7]);
and a63(w[61], a[6], b[7]);
and a64(w[62], a[7], b[7]);

halfadder ha0(w[0], w[7], p[1], c[0]);


halfadder ha1(w[1], w[8], s[0], c[1]);
fulladder fa0(s[0], w[15], c[0], p[2], c[2]);

halfadder ha2(w[2], w[9], s[1], c[3]);
fulladder fa1(s[1], w[16], c[1], s[2], c[4]);
fulladder fa2(s[2], w[23], c[2], p[3], c[5]);


halfadder ha3(w[3], w[10], s[3], c[6]);
fulladder fa3(s[3], w[17], c[3], s[4], c[7]);
fulladder fa4(s[4], w[24], c[4], s[5], c[8]);
fulladder fa5(s[5], w[31], c[5], p[4], c[9]);

halfadder ha4(w[4], w[11], s[6], c[10]);
fulladder fa6(s[6], w[18], c[6], s[7], c[11]);
fulladder fa7(s[7], w[25], c[7], s[8], c[12]);
fulladder fa8(s[8], w[32], c[8], s[9], c[13]);
fulladder fa9(s[9], w[39], c[9], p[5], c[14]);


halfadder ha5(w[5], w[12], s[10], c[15]);
fulladder fa10(s[10], w[19], c[10], s[11], c[16]);
fulladder fa11(s[11], w[26], c[11], s[12], c[17]);
fulladder fa12(s[12], w[33], c[12], s[13], c[18]);
fulladder fa13(s[13], w[40], c[13], s[14], c[19]);
fulladder fa14(s[14], w[47], c[14], p[6], c[20]);



halfadder ha6(w[6], w[13], s[15], c[21]);
fulladder fa15(s[15], w[20], c[15], s[16], c[22]);
fulladder fa16(s[16], w[27], c[16], s[17], c[23]);
fulladder fa17(s[17], w[34], c[17], s[18], c[24]);
fulladder fa18(s[18], w[41], c[18], s[19], c[25]);
fulladder fa19(s[19], w[48], c[19], s[20], c[26]);
fulladder fa20(s[20], w[55], c[20], p[7], c[27]);   




halfadder ha7(c[21], w[14], s[21], c[28]);
fulladder fa21(s[21], w[21], c[22], s[22], c[29]);
fulladder fa22(s[22], w[28], c[23], s[23], c[30]);
fulladder fa23(s[23], w[35], c[24], s[24], c[31]);
fulladder fa24(s[24], w[42], c[25], s[25], c[32]);
fulladder fa25(s[25], w[49], c[26], s[26], c[33]);
fulladder fa26(s[26], w[56], c[27], p[8], c[34]);

fulladder fa27(c[28], w[22], c[29], s[27], c[35]);
fulladder fa28(s[27], w[29], c[30], s[28], c[36]);
fulladder fa29(s[28], w[36], c[31], s[29], c[37]);
fulladder fa30(s[29], w[43], c[32], s[30], c[38]);
fulladder fa31(s[30], w[50], c[33], s[31], c[39]);
fulladder fa32(s[31], w[57], c[34], p[9], c[40]);

fulladder fa33(c[35], w[30], c[36], s[32], c[41]);
fulladder fa34(s[32], w[37], c[37], s[33], c[42]);
fulladder fa35(s[33], w[44], c[38], s[34], c[43]);
fulladder fa36(s[34], w[51], c[39], s[35], c[44]);
fulladder fa37(s[35], w[58], c[40], p[10], c[45]);


fulladder fa38(c[41], w[38], c[42], s[36], c[46]);
fulladder fa39(s[36], w[45], c[43], s[37], c[47]);
fulladder fa40(s[37], w[52], c[44], s[38], c[48]);
fulladder fa41(s[38], w[60], c[45], p[11], c[49]);



fulladder fa42(c[46], w[46], c[47], s[39], c[50]);
fulladder fa43(s[39], w[53], c[48], s[40], c[51]);
fulladder fa44(s[40], w[60], c[49], p[12], c[52]);


fulladder fa45(c[50], w[54], c[51], s[41], c[53]);
fulladder fa46(s[41], w[60], c[52], p[13], c[54]);


fulladder fa47(c[53], w[62], c[54], p[14], p[15]);


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
