module se10(clk,rst,write,addr,wdata,pushin,datain,entrophy,pushout,dataout);
 input clk;
 input rst;
 input write;
 input [11:0] addr;
 input [31:0] wdata;
 input pushin; 
 input [7:0] datain; 
 input [31:0] entrophy;
 output  reg pushout;
 output reg [31:0] dataout;
 reg [11:0] addr_1st;
 reg [63:0] poly0_dataout_1st,poly0_dataout_2nd;
 reg [55:0] poly1_dataout_1st,poly1_dataout_2nd;
 reg [83:0] poly2_dataout_1st,poly2_dataout_2nd;
 reg [60:0] poly3_dataout_1st,poly3_dataout_2nd;
 reg [49:0] poly4_dataout_1st,poly4_dataout_2nd;
 reg [6:0]  poly4_tap_1st,poly4_tap_2nd;
 reg [341:0] poly5_dataout_1st,poly5_dataout_2nd;
 reg [146:0] poly6_dataout_1st,poly6_dataout_2nd;
 reg [346:0] poly7_dataout_1st,poly7_dataout_2nd;
 reg [197:0] poly8_dataout_1st,poly8_dataout_2nd;
 reg [42:0] poly9_dataout_1st,poly9_dataout_2nd;
 reg [126:0] poly10_dataout_1st,poly10_dataout_2nd;
 reg [300:0] poly11_dataout_1st,poly11_dataout_2nd;
 reg [194:0] poly12_dataout_1st,poly12_dataout_2nd;
 reg [59:0] poly13_dataout_1st,poly13_dataout_2nd;
 reg [33:0] poly14_dataout_1st,poly14_dataout_2nd;
 reg [85:0] poly15_dataout_1st,poly15_dataout_2nd;
 reg [15:0] pdata_out_3rd,pdata_out_4th;
 reg [4:0] pselect,pselect_1st,pselect_2nd,pselect_3rd;
 reg [4:0] dselect,dselect_1st,dselect_2nd,dselect_3rd;
 reg [63:0] dataselect_dataout,dataselect_dataout_1st,dataselect_dataout_2nd;
 reg [31:0] first_ivalue_3rd,first_ivalue_4th;
 reg [7:0] dataout_1st,dataout_2nd,dataout_3rd;
 reg pushin_1st; 
reg  [31:0] entrophy_1st,entrophy_2nd,entrophy_3rd,wdata_1st,scramble_out;
 reg write_1st;
 wire pushout_3rda,pushout_3rdb,pushout_3rd,pushout_2nd;
 wire pushout_1st,pushoutf;
reg pushin_2nd,pushin_3rd ;

 assign pushout_3rd=pushout_3rda&pushout_3rdb;

always @(posedge clk or posedge rst)  begin
if (rst)
begin 
  write_1st<=0;
  addr_1st<=0;
  wdata_1st<=0;
  entrophy_1st<=0;
  dataout_1st<=0;
/*Second pipeline*/
  entrophy_2nd<=0;
  dataout_2nd<=0;
poly4_tap_2nd<=0;
  poly0_dataout_2nd<=0;
  poly1_dataout_2nd<=0;
  poly2_dataout_2nd<=0;
  poly3_dataout_2nd<=0;
  poly4_dataout_2nd<=0;
  poly5_dataout_2nd<=0;
  poly6_dataout_2nd<=0;
  poly7_dataout_2nd<=0;
  poly8_dataout_2nd<=0;
  poly9_dataout_2nd<=0;
  poly10_dataout_2nd<=0;
  poly11_dataout_2nd<=0;
  poly12_dataout_2nd<=0;
  poly13_dataout_2nd<=0;
  poly14_dataout_2nd<=0;
  poly15_dataout_2nd<=0;
  pdata_out_4th<=0;
  entrophy_3rd<=0;
  dataout_3rd<=0;
  pushin_1st<=0;
dselect_3rd<=0;
pselect_3rd<=0;
pushin_2nd<=0;
first_ivalue_4th<=0;
pushin_3rd<=0;
pushout<=0;
  poly4_tap_2nd<=0;
end
else
 begin
/* first pipeline input signals*/
  write_1st<=  write;
  addr_1st<=  addr;
  wdata_1st<=  wdata;
  entrophy_1st<=  entrophy;
  dataout_1st<=  datain;
/*Second pipeline*/
  entrophy_2nd<=  entrophy_1st;
  dataout_2nd<=  dataout_1st;
  poly4_tap_2nd<=  poly4_tap_1st;

  poly0_dataout_2nd<= poly0_dataout_1st;
  poly1_dataout_2nd<= poly1_dataout_1st;
  poly2_dataout_2nd<= poly2_dataout_1st;
  poly3_dataout_2nd<= poly3_dataout_1st;
  poly4_dataout_2nd<= poly4_dataout_1st;
  poly5_dataout_2nd<= poly5_dataout_1st;
  poly6_dataout_2nd<= poly6_dataout_1st;
  poly7_dataout_2nd<= poly7_dataout_1st;
  poly8_dataout_2nd<= poly8_dataout_1st;
  poly9_dataout_2nd<= poly9_dataout_1st;
  poly10_dataout_2nd<= poly10_dataout_1st;
  poly11_dataout_2nd<= poly11_dataout_1st;
  poly12_dataout_2nd<= poly12_dataout_1st;
  poly13_dataout_2nd<= poly13_dataout_1st;
  poly14_dataout_2nd<= poly14_dataout_1st;
  poly15_dataout_2nd<= poly15_dataout_1st;

/* fourth pipeline*/
entrophy_3rd<= entrophy_2nd;
dataout_3rd<= dataout_2nd;
pdata_out_4th<= pdata_out_3rd;
pushin_1st<= pushin;
dselect_3rd<= dselect_2nd;
pselect_3rd<= pselect_2nd;
pushin_2nd<= pushin_1st;
first_ivalue_4th<= first_ivalue_3rd;
pushout<= pushoutf;
end
end
 poly0 pl0(clk,rst,addr,write,pushin,wdata,poly0_dataout_1st);
 poly1 pl1(clk,rst,addr,write,pushin,wdata,poly1_dataout_1st);
 poly2 pl2(clk,rst,addr,write,pushin,wdata,poly2_dataout_1st);
 poly3 pl3(clk,rst,addr,write,pushin,wdata,poly3_dataout_1st);
 poly4 pl4(clk,rst,addr,write,pushin,wdata,poly4_dataout_1st,poly4_tap_1st);
 poly5 pl5(clk,rst,addr,write,pushin,wdata,poly5_dataout_1st);
 poly6 pl6(clk,rst,addr,write,pushin,wdata,poly6_dataout_1st);
 poly7 pl7(clk,rst,addr,write,pushin,wdata,poly7_dataout_1st);
 poly8 pl8(clk,rst,addr,write,pushin,wdata,poly8_dataout_1st);
 poly9 pl9(clk,rst,addr,write,pushin,wdata,poly9_dataout_1st);
 poly10 pl10(clk,rst,addr,write,pushin,wdata,poly10_dataout_1st);
 poly11 pl11(clk,rst,addr,write,pushin,wdata,poly11_dataout_1st);
 poly12 pl12(clk,rst,addr,write,pushin,wdata,poly12_dataout_1st);
 poly13 pl13(clk,rst,addr,write,pushin,wdata,poly13_dataout_1st);
 poly14 pl14(clk,rst,addr,write,pushin,wdata,poly14_dataout_1st);
 poly15 pl15(clk,rst,addr,write,pushin,wdata,poly15_dataout_1st);

 data_selector datasel (clk,rst,addr_1st,write_1st,pushin_1st,wdata_1st,poly4_tap_2nd,dataselect_dataout_2nd,dselect_2nd,pselect_2nd);

decoder_entrophy decoder2(clk,rst,pushin_1st,dselect_2nd,dataout_1st,entrophy_1st,first_ivalue_3rd,pushout_3rda);

decoder_plfsr decoder1 (clk,rst,pushin_1st,pselect_2nd,poly0_dataout_2nd,poly1_dataout_2nd,poly2_dataout_2nd,poly3_dataout_2nd,poly4_dataout_2nd,poly5_dataout_2nd     ,poly6_dataout_2nd,poly7_dataout_2nd,poly8_dataout_2nd,poly9_dataout_2nd,poly10_dataout_2nd,poly11_dataout_2nd,poly12_dataout_2nd,poly13_dataout_2nd,poly14_dataout_2nd,poly15_dataout_2nd,pdata_out_3rd,pushout_3rdb);

data_scrambler datascram (clk,rst,pushin_2nd,pdata_out_4th,first_ivalue_4th,dataout,pushoutf);
endmodule

// Code your design here
module poly0(clk,rst,addr,write,pushin,wdata,poly0_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write;
 input pushin;
 input [31:0] wdata;
 output  reg [63:0] poly0_dataout;

always @(posedge clk or posedge rst) begin
 if(rst) 
    begin   
     poly0_dataout<=64'b0;
        end
   else
    begin 
    if((write==1'b1) &&(addr==12'h18))
          begin
           poly0_dataout[31:0]=wdata;
          end  
         if((write==1'b1)&&(addr==12'h19))
          begin
           poly0_dataout[63:32]=wdata;
	end
 if(pushin) begin
 for (int i=0;i<18;i=i+1) begin
 poly0_dataout={poly0_dataout[62],poly0_dataout[61],poly0_dataout[60]^poly0_dataout[63],poly0_dataout[59],poly0_dataout[58],poly0_dataout[57],poly0_dataout[56],poly0_dataout[55],poly0_dataout[54],poly0_dataout[53],poly0_dataout[52],poly0_dataout[51],poly0_dataout[50],poly0_dataout[49],poly0_dataout[48],poly0_dataout[47],poly0_dataout[46],poly0_dataout[45],poly0_dataout[44],poly0_dataout[43],poly0_dataout[42],poly0_dataout[41],poly0_dataout[40],poly0_dataout[39],poly0_dataout[38],poly0_dataout[37],poly0_dataout[36],poly0_dataout[35],poly0_dataout[34],poly0_dataout[33]^poly0_dataout[63],poly0_dataout[32],poly0_dataout[31],poly0_dataout[30],poly0_dataout[29],poly0_dataout[28],poly0_dataout[27],poly0_dataout[26],poly0_dataout[25],poly0_dataout[24],poly0_dataout[23],poly0_dataout[22],poly0_dataout[21],poly0_dataout[20],poly0_dataout[19],poly0_dataout[18],poly0_dataout[17],poly0_dataout[16],poly0_dataout[15],poly0_dataout[14],poly0_dataout[13],poly0_dataout[12],poly0_dataout[11],poly0_dataout[10],poly0_dataout[09],poly0_dataout[08]^poly0_dataout[63],poly0_dataout[07],poly0_dataout[06],poly0_dataout[05],poly0_dataout[04],poly0_dataout[03],poly0_dataout[02],poly0_dataout[01],poly0_dataout[00],poly0_dataout[63]};
	  end
	//$display ("for value of poly0_dataout is %h\n",poly0_dataout);
	  end
	  end
	  end
endmodule 
	 
	 
	
	 
	  
	  
	 
	 


 


module poly10(clk,rst,addr,write,pushin,wdata,poly10_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [126:0] poly10_dataout;
 reg [126:0] p10 ;
 
	
 always @(posedge clk or posedge rst)
 begin
        if(rst)
        begin
        poly10_dataout<=127'b0;
        end
else
    begin
        if((write==1'b1) &&(addr==12'h06a))
          begin
          poly10_dataout[31:0]=wdata;
          end
        else if((write==1'b1)&&(addr==12'h06b))
          begin
           poly10_dataout[63:32]=wdata;
          end
        else if((write==1'b1)&&(addr==12'h06c))
          begin
           poly10_dataout[95:64]=wdata;
           end
          else  if((write==1'b1)&&(addr==12'h06d))
          begin
           poly10_dataout[126:96]=wdata;
           end
	if(pushin) begin
        for (int i=0;i<13;i=i+1) 
	 begin
	   poly10_dataout={poly10_dataout[125],poly10_dataout[124],poly10_dataout[123],poly10_dataout[122],poly10_dataout[121],poly10_dataout[120],poly10_dataout[119],poly10_dataout[118],poly10_dataout[117],poly10_dataout[116],poly10_dataout[115],poly10_dataout[114],poly10_dataout[113],poly10_dataout[112],poly10_dataout[111],poly10_dataout[110],poly10_dataout[109],poly10_dataout[108],poly10_dataout[107],poly10_dataout[106],poly10_dataout[105],poly10_dataout[104],poly10_dataout[103],poly10_dataout[102],poly10_dataout[101],poly10_dataout[100],poly10_dataout[99],poly10_dataout[98],poly10_dataout[97],poly10_dataout[96],poly10_dataout[95],poly10_dataout[94],poly10_dataout[93],poly10_dataout[92],poly10_dataout[91],poly10_dataout[90],poly10_dataout[89],poly10_dataout[88],poly10_dataout[87],poly10_dataout[86],poly10_dataout[85],poly10_dataout[84],poly10_dataout[83],poly10_dataout[82],poly10_dataout[81],poly10_dataout[80],poly10_dataout[79],poly10_dataout[78],poly10_dataout[77],poly10_dataout[76],poly10_dataout[75],poly10_dataout[74],poly10_dataout[73],poly10_dataout[72],poly10_dataout[71],poly10_dataout[70],poly10_dataout[69],poly10_dataout[68],poly10_dataout[67],poly10_dataout[66],poly10_dataout[65],poly10_dataout[64],poly10_dataout[63],poly10_dataout[62],poly10_dataout[61],poly10_dataout[60],poly10_dataout[59],poly10_dataout[58],poly10_dataout[57],poly10_dataout[56],poly10_dataout[55],poly10_dataout[54],poly10_dataout[53]^poly10_dataout[126],poly10_dataout[52],poly10_dataout[51],poly10_dataout[50],poly10_dataout[49],poly10_dataout[48],poly10_dataout[47],poly10_dataout[46],poly10_dataout[45],poly10_dataout[44]^poly10_dataout[126],poly10_dataout[43],poly10_dataout[42],poly10_dataout[41],poly10_dataout[40],poly10_dataout[39],poly10_dataout[38],poly10_dataout[37],poly10_dataout[36],poly10_dataout[35],poly10_dataout[34],poly10_dataout[33],poly10_dataout[32],poly10_dataout[31],poly10_dataout[30],poly10_dataout[29],poly10_dataout[28],poly10_dataout[27],poly10_dataout[26],poly10_dataout[25],poly10_dataout[24],poly10_dataout[23],poly10_dataout[22],poly10_dataout[21],poly10_dataout[20],poly10_dataout[19],poly10_dataout[18],poly10_dataout[17],poly10_dataout[16],poly10_dataout[15],poly10_dataout[14],poly10_dataout[13],poly10_dataout[12]^poly10_dataout[126],poly10_dataout[11],poly10_dataout[10],poly10_dataout[9],poly10_dataout[8],poly10_dataout[7],poly10_dataout[6],poly10_dataout[5],poly10_dataout[4],poly10_dataout[3],poly10_dataout[2],poly10_dataout[1],poly10_dataout[0],poly10_dataout[126]};
	  end
	  end
	  end
	  end

endmodule 


module poly11(clk,rst,addr,write,pushin,wdata,poly11_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [300:0] poly11_dataout;
            

always @(posedge clk or posedge rst)
begin
	   if(rst)
	   begin
	   poly11_dataout<=301'b0;
	   end
    else begin
        if((write==1'b1) &&(addr==12'h06f))
          begin
          poly11_dataout[31:0]=wdata;
          end
         else if((write==1'b1)&&(addr==12'h070))
          begin
           poly11_dataout[63:32]=wdata;
          end
        else if((write==1'b1)&&(addr==12'h071))
          begin
           poly11_dataout[95:64]=wdata;
           end
           else if((write==1'b1)&&(addr==12'h072))
          begin
           poly11_dataout[127:96]=wdata;
           end
           else if((write==1'b1)&&(addr==12'h073))
          begin
           poly11_dataout[159:128]=wdata;
           end
              else if((write==1'b1)&&(addr==12'h074))
          begin
           poly11_dataout[191:160]=wdata;
           end
              else if((write==1'b1)&&(addr==12'h075))
          begin
           poly11_dataout[223:192]=wdata;
           end
              else if((write==1'b1)&&(addr==12'h076))
          begin
           poly11_dataout[255:224]=wdata;
           end
              else if((write==1'b1)&&(addr==12'h077))
          begin
           poly11_dataout[287:256]=wdata;
           end
              else if((write==1'b1)&&(addr==12'h078))
          begin
           poly11_dataout[300:288]=wdata;
           end

if(pushin) begin
        for (int i=0;i<15;i=i+1) 
            begin
        poly11_dataout={ poly11_dataout[299],poly11_dataout[298],poly11_dataout[297],poly11_dataout[296],poly11_dataout[295],poly11_dataout[294],poly11_dataout[293],poly11_dataout[292],poly11_dataout[291],poly11_dataout[290],poly11_dataout[289],poly11_dataout[288],poly11_dataout[287],poly11_dataout[286],poly11_dataout[285],poly11_dataout[284],poly11_dataout[283],poly11_dataout[282],poly11_dataout[281],poly11_dataout[280],poly11_dataout[279],poly11_dataout[278],poly11_dataout[277],poly11_dataout[276]^poly11_dataout[300],poly11_dataout[275],poly11_dataout[274],poly11_dataout[273],poly11_dataout[272],poly11_dataout[271],poly11_dataout[270],poly11_dataout[269],poly11_dataout[268],poly11_dataout[267],poly11_dataout[266],poly11_dataout[265],poly11_dataout[264],poly11_dataout[263],poly11_dataout[262],poly11_dataout[261],poly11_dataout[260],poly11_dataout[259],poly11_dataout[258],poly11_dataout[257],poly11_dataout[256],poly11_dataout[255],poly11_dataout[254],poly11_dataout[253],poly11_dataout[252],poly11_dataout[251],poly11_dataout[250],poly11_dataout[249],poly11_dataout[248],poly11_dataout[247],poly11_dataout[246],poly11_dataout[245],poly11_dataout[244],poly11_dataout[243],poly11_dataout[242],poly11_dataout[241],poly11_dataout[240],poly11_dataout[239],poly11_dataout[238],poly11_dataout[237],poly11_dataout[236],poly11_dataout[235],poly11_dataout[234],poly11_dataout[233],poly11_dataout[232],poly11_dataout[231],poly11_dataout[230],poly11_dataout[229],poly11_dataout[228],poly11_dataout[227],poly11_dataout[226],poly11_dataout[225],poly11_dataout[224],poly11_dataout[223],poly11_dataout[222],poly11_dataout[221]^poly11_dataout[300],poly11_dataout[220],poly11_dataout[219],poly11_dataout[218],poly11_dataout[217],poly11_dataout[216],poly11_dataout[215],poly11_dataout[214],poly11_dataout[213],poly11_dataout[212],poly11_dataout[211],poly11_dataout[210],poly11_dataout[209]^poly11_dataout[300],poly11_dataout[208],poly11_dataout[207],poly11_dataout[206],poly11_dataout[205],poly11_dataout[204],poly11_dataout[203],poly11_dataout[202],poly11_dataout[201],poly11_dataout[200],poly11_dataout[199],poly11_dataout[198],poly11_dataout[197],poly11_dataout[196],poly11_dataout[195]^ poly11_dataout[300],poly11_dataout[194],poly11_dataout[193],poly11_dataout[192],poly11_dataout[191],poly11_dataout[190],poly11_dataout[189],poly11_dataout[188],poly11_dataout[187],poly11_dataout[186],poly11_dataout[185],poly11_dataout[184],poly11_dataout[183],poly11_dataout[182],poly11_dataout[181],poly11_dataout[180],poly11_dataout[179],poly11_dataout[178],poly11_dataout[177],poly11_dataout[176],poly11_dataout[175],poly11_dataout[174],poly11_dataout[173],poly11_dataout[172],poly11_dataout[171],poly11_dataout[170],poly11_dataout[169],poly11_dataout[168],poly11_dataout[167],poly11_dataout[166],poly11_dataout[165],poly11_dataout[164],poly11_dataout[163],poly11_dataout[162],poly11_dataout[161],poly11_dataout[160],poly11_dataout[159],poly11_dataout[158],poly11_dataout[157],poly11_dataout[156],poly11_dataout[155],poly11_dataout[154],poly11_dataout[153],poly11_dataout[152],poly11_dataout[151],poly11_dataout[150],poly11_dataout[149],poly11_dataout[148],poly11_dataout[147],poly11_dataout[146],poly11_dataout[145],poly11_dataout[144],poly11_dataout[143],poly11_dataout[142],poly11_dataout[141],poly11_dataout[140],poly11_dataout[139],poly11_dataout[138],poly11_dataout[137],poly11_dataout[136],poly11_dataout[135],poly11_dataout[134],poly11_dataout[133],poly11_dataout[132],poly11_dataout[131],poly11_dataout[130],poly11_dataout[129],poly11_dataout[128],poly11_dataout[127],poly11_dataout[126],poly11_dataout[125],poly11_dataout[124],poly11_dataout[123],poly11_dataout[122],poly11_dataout[121],poly11_dataout[120],poly11_dataout[119],poly11_dataout[118],poly11_dataout[117],poly11_dataout[116],poly11_dataout[115],poly11_dataout[114],poly11_dataout[113],poly11_dataout[112],poly11_dataout[111],poly11_dataout[110],poly11_dataout[109],poly11_dataout[108],poly11_dataout[107],poly11_dataout[106],poly11_dataout[105],poly11_dataout[104],poly11_dataout[103],poly11_dataout[102],poly11_dataout[101],poly11_dataout[100],poly11_dataout[99],poly11_dataout[98],poly11_dataout[97],poly11_dataout[96],poly11_dataout[95],poly11_dataout[94],poly11_dataout[93],poly11_dataout[92],poly11_dataout[91],poly11_dataout[90],poly11_dataout[89],poly11_dataout[88],poly11_dataout[87],poly11_dataout[86],poly11_dataout[85],poly11_dataout[84],poly11_dataout[83],poly11_dataout[82],poly11_dataout[81],poly11_dataout[80],poly11_dataout[79],poly11_dataout[78],poly11_dataout[77],poly11_dataout[76],poly11_dataout[75],poly11_dataout[74],poly11_dataout[73],poly11_dataout[72],poly11_dataout[71],poly11_dataout[70],poly11_dataout[69],poly11_dataout[68],poly11_dataout[67],poly11_dataout[66],poly11_dataout[65],poly11_dataout[64],poly11_dataout[63],poly11_dataout[62],poly11_dataout[61],poly11_dataout[60],poly11_dataout[59],poly11_dataout[58],poly11_dataout[57],poly11_dataout[56],poly11_dataout[55],poly11_dataout[54],poly11_dataout[53],poly11_dataout[52],poly11_dataout[51],poly11_dataout[50],poly11_dataout[49],poly11_dataout[48],poly11_dataout[47],poly11_dataout[46],poly11_dataout[45],poly11_dataout[44],poly11_dataout[43],poly11_dataout[42],poly11_dataout[41],poly11_dataout[40],poly11_dataout[39],poly11_dataout[38],poly11_dataout[37],poly11_dataout[36],poly11_dataout[35],poly11_dataout[34],poly11_dataout[33],poly11_dataout[32]^poly11_dataout[300],poly11_dataout[31],poly11_dataout[30],poly11_dataout[29],poly11_dataout[28],poly11_dataout[27],poly11_dataout[26],poly11_dataout[25],poly11_dataout[24],poly11_dataout[23],poly11_dataout[22],poly11_dataout[21],poly11_dataout[20],poly11_dataout[19],poly11_dataout[18],poly11_dataout[17],poly11_dataout[16],poly11_dataout[15],poly11_dataout[14],poly11_dataout[13],poly11_dataout[12],poly11_dataout[11],poly11_dataout[10],poly11_dataout[9],poly11_dataout[8],poly11_dataout[7],poly11_dataout[6],poly11_dataout[5],poly11_dataout[4],poly11_dataout[3],poly11_dataout[2],poly11_dataout[1],poly11_dataout[0],poly11_dataout[300]};
end
// $display ("for value of poly11_dataout is %h\n",poly11_dataout);
end
end
end
endmodule 


module poly12(clk,rst,addr,write,pushin,wdata,poly12_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [194:0] poly12_dataout;
 reg [194:0] p12 ;

always @(posedge clk or posedge rst)
begin
	   if(rst)
	   begin
	   poly12_dataout<=195'b0;
	   end
    else
        begin
      if((write==1'b1) &&(addr==12'h07e))
          begin
          poly12_dataout[31:0]=wdata;
          end  
          else if((write==1'b1)&&(addr==12'h07f))
          begin
           poly12_dataout[63:32]=wdata;
          end  
         else if((write==1'b1)&&(addr==12'h080))
          begin
           poly12_dataout[95:64]=wdata;
           end  
            else if((write==1'b1)&&(addr==12'h081))
          begin
           poly12_dataout[127:96]=wdata;
           end  
            else if((write==1'b1)&&(addr==12'h082))
          begin
           poly12_dataout[159:128]=wdata;
           end  
               else if((write==1'b1)&&(addr==12'h083))
          begin
           poly12_dataout[191:160]=wdata;
           end  
               else if((write==1'b1)&&(addr==12'h084))
          begin
           poly12_dataout[194:192]=wdata;
           end  

	if(pushin) begin
        for (int i=0;i<12;i=i+1) 
            begin
        poly12_dataout={poly12_dataout[193],poly12_dataout[192],poly12_dataout[191],poly12_dataout[190],poly12_dataout[189],poly12_dataout[188],poly12_dataout[187],poly12_dataout[186],poly12_dataout[185],poly12_dataout[184],poly12_dataout[183],poly12_dataout[182],poly12_dataout[181],poly12_dataout[180],poly12_dataout[179],poly12_dataout[178],poly12_dataout[177],poly12_dataout[176],poly12_dataout[175],poly12_dataout[174],poly12_dataout[173],poly12_dataout[172],poly12_dataout[171],poly12_dataout[170],poly12_dataout[169],poly12_dataout[168],poly12_dataout[167],poly12_dataout[166],poly12_dataout[165],poly12_dataout[164],poly12_dataout[163],poly12_dataout[162],poly12_dataout[161],poly12_dataout[160],poly12_dataout[159],poly12_dataout[158],poly12_dataout[157],poly12_dataout[156],poly12_dataout[155],poly12_dataout[154],poly12_dataout[153]^poly12_dataout[194],poly12_dataout[152],poly12_dataout[151],poly12_dataout[150],poly12_dataout[149],poly12_dataout[148],poly12_dataout[147],poly12_dataout[146],poly12_dataout[145],poly12_dataout[144],poly12_dataout[143],poly12_dataout[142],poly12_dataout[141],poly12_dataout[140],poly12_dataout[139],poly12_dataout[138],poly12_dataout[137],poly12_dataout[136],poly12_dataout[135],poly12_dataout[134],poly12_dataout[133],poly12_dataout[132],poly12_dataout[131],poly12_dataout[130],poly12_dataout[129],poly12_dataout[128],poly12_dataout[127],poly12_dataout[126],poly12_dataout[125],poly12_dataout[124],poly12_dataout[123],poly12_dataout[122],poly12_dataout[121],poly12_dataout[120],poly12_dataout[119],poly12_dataout[118],poly12_dataout[117],poly12_dataout[116],poly12_dataout[115],poly12_dataout[114],poly12_dataout[113],poly12_dataout[112],poly12_dataout[111],poly12_dataout[110],poly12_dataout[109],poly12_dataout[108],poly12_dataout[107]^poly12_dataout[194],poly12_dataout[106],poly12_dataout[105]^poly12_dataout[194],poly12_dataout[104]^poly12_dataout[194],poly12_dataout[103],poly12_dataout[102],poly12_dataout[101],poly12_dataout[100],poly12_dataout[99],poly12_dataout[98],poly12_dataout[97],poly12_dataout[96],poly12_dataout[95],poly12_dataout[94],poly12_dataout[93],poly12_dataout[92],poly12_dataout[91],poly12_dataout[90],poly12_dataout[89],poly12_dataout[88],poly12_dataout[87],poly12_dataout[86],poly12_dataout[85],poly12_dataout[84],poly12_dataout[83]^poly12_dataout[194],poly12_dataout[82],poly12_dataout[81],poly12_dataout[80],poly12_dataout[79],poly12_dataout[78],poly12_dataout[77],poly12_dataout[76],poly12_dataout[75],poly12_dataout[74],poly12_dataout[73],poly12_dataout[72],poly12_dataout[71],poly12_dataout[70],poly12_dataout[69],poly12_dataout[68],poly12_dataout[67],poly12_dataout[66],poly12_dataout[65],poly12_dataout[64],poly12_dataout[63],poly12_dataout[62],poly12_dataout[61],poly12_dataout[60],poly12_dataout[59],poly12_dataout[58],poly12_dataout[57],poly12_dataout[56],poly12_dataout[55],poly12_dataout[54],poly12_dataout[53],poly12_dataout[52],poly12_dataout[51],poly12_dataout[50],poly12_dataout[49],poly12_dataout[48],poly12_dataout[47],poly12_dataout[46],poly12_dataout[45],poly12_dataout[44],poly12_dataout[43],poly12_dataout[42],poly12_dataout[41],poly12_dataout[40],poly12_dataout[39],poly12_dataout[38],poly12_dataout[37],poly12_dataout[36],poly12_dataout[35],poly12_dataout[34],poly12_dataout[33],poly12_dataout[32],poly12_dataout[31],poly12_dataout[30],poly12_dataout[29],poly12_dataout[28],poly12_dataout[27],poly12_dataout[26],poly12_dataout[25],poly12_dataout[24],poly12_dataout[23],poly12_dataout[22],poly12_dataout[21],poly12_dataout[20],poly12_dataout[19],poly12_dataout[18],poly12_dataout[17],poly12_dataout[16],poly12_dataout[15],poly12_dataout[14],poly12_dataout[13],poly12_dataout[12],poly12_dataout[11],poly12_dataout[10],poly12_dataout[9],poly12_dataout[8],poly12_dataout[7],poly12_dataout[6],poly12_dataout[5],poly12_dataout[4],poly12_dataout[3],poly12_dataout[2],poly12_dataout[1],poly12_dataout[0],poly12_dataout[194]};
	  end
	  end
 //$display ("for value of poly12_dataout is %h\n",poly12_dataout);
	  end
	  end
endmodule 


module poly13(clk,rst,addr,write,pushin,wdata,poly13_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [59:0] poly13_dataout;
 reg [59:0] p13 ;

always @(posedge clk or posedge rst)
begin
	   if(rst)
	   begin
	   poly13_dataout<=60'b0;
	   end
    else
        begin
      if((write==1'b1) &&(addr==12'h088))
          begin
          poly13_dataout[31:0]=wdata;
          end  
         else if((write==1'b1)&&(addr==12'h089))
          begin
           poly13_dataout[59:32]=wdata;
          end  
	else if(pushin) begin
        for (int i=0;i<11;i=i+1) 
            begin
        poly13_dataout={ poly13_dataout[58],poly13_dataout[57],poly13_dataout[56],poly13_dataout[55],poly13_dataout[54],poly13_dataout[53],poly13_dataout[52],poly13_dataout[51],poly13_dataout[50],poly13_dataout[49],poly13_dataout[48],poly13_dataout[47]^poly13_dataout[59],poly13_dataout[46],poly13_dataout[45],poly13_dataout[44],poly13_dataout[43],poly13_dataout[42],poly13_dataout[41],poly13_dataout[40],poly13_dataout[39],poly13_dataout[38],poly13_dataout[37],poly13_dataout[36],poly13_dataout[35],poly13_dataout[34],poly13_dataout[33],poly13_dataout[32],poly13_dataout[31],poly13_dataout[30]^poly13_dataout[59],poly13_dataout[29],poly13_dataout[28],poly13_dataout[27],poly13_dataout[26],poly13_dataout[25],poly13_dataout[24],poly13_dataout[23],poly13_dataout[22],poly13_dataout[21],poly13_dataout[20],poly13_dataout[19],poly13_dataout[18]^poly13_dataout[59],poly13_dataout[17],poly13_dataout[16],poly13_dataout[15],poly13_dataout[14],poly13_dataout[13],poly13_dataout[12]^poly13_dataout[59],poly13_dataout[11]^poly13_dataout[59],poly13_dataout[10],poly13_dataout[9],poly13_dataout[8],poly13_dataout[7],poly13_dataout[6],poly13_dataout[5],poly13_dataout[4],poly13_dataout[3],poly13_dataout[2],poly13_dataout[1],poly13_dataout[0],poly13_dataout[59]};
	  end
 // $display ("for value of poly13_dataout is %h\n",poly13_dataout);
	  end
	  end
	  end
endmodule 


module poly14(clk,rst,addr,write,pushin,wdata,poly14_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [33:0] poly14_dataout;
 reg [33:0] p14 ;
 wire [33:0] poly14_dataout_d;
 

always @(posedge clk or posedge rst)
begin
	   if(rst)
	   begin
	   poly14_dataout<=34'b0;
	   end
    else
        begin
      if((write==1'b1) &&(addr==12'h091))
          begin
          poly14_dataout[31:0]=wdata;
          end
        else  if((write==1'b1)&&(addr==12'h092))
          begin
           poly14_dataout[33:32]=wdata;
          end
	if(pushin) begin
        for (int i=0;i<15;i=i+1) 
            begin
        poly14_dataout={poly14_dataout[32],poly14_dataout[31],poly14_dataout[30],poly14_dataout[29],poly14_dataout[28],poly14_dataout[27],poly14_dataout[26],poly14_dataout[25],poly14_dataout[24],poly14_dataout[23],poly14_dataout[22],poly14_dataout[21],poly14_dataout[20],poly14_dataout[19],poly14_dataout[18],poly14_dataout[17],poly14_dataout[16]^poly14_dataout[33],poly14_dataout[15],poly14_dataout[14],poly14_dataout[13],poly14_dataout[12],poly14_dataout[11]^poly14_dataout[33],poly14_dataout[10],poly14_dataout[9],poly14_dataout[8],poly14_dataout[7]^poly14_dataout[33],poly14_dataout[6],poly14_dataout[5],poly14_dataout[4],poly14_dataout[3],poly14_dataout[2],poly14_dataout[1],poly14_dataout[0],poly14_dataout[33]}; 
	  end
 //$display ("for value of poly14_dataout is %h\n",poly14_dataout);
	  end
	  end
	  end
endmodule 


module poly15(clk,rst,addr,write,pushin,wdata,poly15_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [85:0] poly15_dataout;
 

always @(posedge clk or posedge rst)
begin
	   if(rst)
	   begin
	   poly15_dataout<=86'b0;
	   end
    else
        begin
      if((write==1'b1) &&(addr==12'h093))
          begin
          poly15_dataout[31:0]=wdata;
          end
     else  if((write==1'b1)&&(addr==12'h094))
          begin
           poly15_dataout[63:32]=wdata;
          end
       else  if((write==1'b1)&&(addr==12'h095))
          begin
           poly15_dataout[85:64]=wdata;
           end
if(pushin) begin
for (int i=0;i<10;i=i+1) 
            begin
        poly15_dataout={poly15_dataout[84],poly15_dataout[83],poly15_dataout[82],poly15_dataout[81],poly15_dataout[80],poly15_dataout[79]^poly15_dataout[85],poly15_dataout[78],poly15_dataout[77],poly15_dataout[76],poly15_dataout[75],poly15_dataout[74],poly15_dataout[73],poly15_dataout[72],poly15_dataout[71],poly15_dataout[70],poly15_dataout[69],poly15_dataout[68],poly15_dataout[67],poly15_dataout[66],poly15_dataout[65],poly15_dataout[64],poly15_dataout[63],poly15_dataout[62],poly15_dataout[61],poly15_dataout[60],poly15_dataout[59],poly15_dataout[58],poly15_dataout[57],poly15_dataout[56],poly15_dataout[55],poly15_dataout[54],poly15_dataout[53],poly15_dataout[52],poly15_dataout[51],poly15_dataout[50],poly15_dataout[49],poly15_dataout[48],poly15_dataout[47],poly15_dataout[46],poly15_dataout[45],poly15_dataout[44],poly15_dataout[43],poly15_dataout[42],poly15_dataout[41],poly15_dataout[40],poly15_dataout[39],poly15_dataout[38],poly15_dataout[37],poly15_dataout[36],poly15_dataout[35],poly15_dataout[34],poly15_dataout[33],poly15_dataout[32],poly15_dataout[31],poly15_dataout[30],poly15_dataout[29],poly15_dataout[28],poly15_dataout[27],poly15_dataout[26],poly15_dataout[25],poly15_dataout[24],poly15_dataout[23],poly15_dataout[22],poly15_dataout[21],poly15_dataout[20],poly15_dataout[19],poly15_dataout[18],poly15_dataout[17],poly15_dataout[16],poly15_dataout[15],poly15_dataout[14],poly15_dataout[13],poly15_dataout[12],poly15_dataout[11],poly15_dataout[10],poly15_dataout[9]^poly15_dataout[85],poly15_dataout[8],poly15_dataout[7],poly15_dataout[6]^poly15_dataout[85],poly15_dataout[5],poly15_dataout[4],poly15_dataout[3],poly15_dataout[2],poly15_dataout[1],poly15_dataout[0],poly15_dataout[85]};
	end
 //$display ("for value of poly15_dataout is %h\n",poly15_dataout);
	end
	end
	end
endmodule 
// Code your design here
module poly1(clk,rst,addr,write,pushin,wdata,poly1_dataout);
 input clk;
 input rst;
input pushin;
 input [11:0] addr;
 input [31:0] wdata;
 input write;
 output reg [55:0] poly1_dataout;


 always @(posedge clk or posedge rst)
 begin
   if(rst)
    begin
      poly1_dataout<=64'b0;
	end
   else
    begin

          if((write==1'b1) &&(addr==12'h21))
          begin
                 poly1_dataout[31:0]=wdata;
          end
         else  if((write==1'b1)&&(addr==12'h22))
          begin
                poly1_dataout[55:32]=wdata;
          end
	if(pushin) begin
	for (int i=0;i<17;i=i+1) 
	 begin
	   poly1_dataout={
	          poly1_dataout[54],poly1_dataout[53],poly1_dataout[52],poly1_dataout[51],
	          poly1_dataout[50],poly1_dataout[49],poly1_dataout[48],poly1_dataout[47],poly1_dataout[46],poly1_dataout[45],poly1_dataout[44]^poly1_dataout[55],poly1_dataout[43],poly1_dataout[42],poly1_dataout[41],poly1_dataout[40],
	          poly1_dataout[39],poly1_dataout[38],poly1_dataout[37]^poly1_dataout[55],poly1_dataout[36],poly1_dataout[35],poly1_dataout[34],poly1_dataout[33],poly1_dataout[32],poly1_dataout[31],poly1_dataout[30],
	          poly1_dataout[29],poly1_dataout[28],poly1_dataout[27]^poly1_dataout[55],poly1_dataout[26],poly1_dataout[25],poly1_dataout[24],poly1_dataout[23],poly1_dataout[22],poly1_dataout[21],poly1_dataout[20],
	          poly1_dataout[19]^poly1_dataout[55],poly1_dataout[18],poly1_dataout[17],poly1_dataout[16],poly1_dataout[15],poly1_dataout[14],poly1_dataout[13],poly1_dataout[12],poly1_dataout[11],poly1_dataout[10],
	          poly1_dataout[09],poly1_dataout[08],poly1_dataout[07],poly1_dataout[06],poly1_dataout[05],poly1_dataout[04]^poly1_dataout[55],poly1_dataout[03],poly1_dataout[02],poly1_dataout[01],poly1_dataout[00],poly1_dataout[55]};
	  end
//$display ("for value of poly1_dataout is %h\n",poly1_dataout);
	  end
	  end
	  end

endmodule 
	 
	 
	
	 
	  
	  
	 
	 
 
module poly2(clk,rst,addr,write,pushin,wdata,poly2_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output  reg [83:0] poly2_dataout;
 
always @(posedge clk or posedge rst) begin
   if(rst)
    begin
      poly2_dataout<=64'b0;
	end
   else
    begin
	
  if((write==1'b1) &&(addr==12'h26))
          begin
           poly2_dataout[31:0]=wdata;
          end
        else  if((write==1'b1)&&(addr==12'h27))
	   begin
           poly2_dataout[63:32]=wdata;
          end
          else if((write==1'b1)&&(addr==12'h28))
          begin
           poly2_dataout[83:64]=wdata;
          end
	if(pushin) begin
	for (int i=0;i<12;i=i+1) 
	 begin
	   poly2_dataout={  poly2_dataout[82],poly2_dataout[81]^poly2_dataout[83],poly2_dataout[80],poly2_dataout[79],poly2_dataout[78],poly2_dataout[77],poly2_dataout[76],poly2_dataout[75],poly2_dataout[74],poly2_dataout[73],poly2_dataout[72],poly2_dataout[71],poly2_dataout[70],poly2_dataout[69],poly2_dataout[68],poly2_dataout[67],poly2_dataout[66],poly2_dataout[65],poly2_dataout[64],poly2_dataout[63],poly2_dataout[62],poly2_dataout[61]^poly2_dataout[83],poly2_dataout[60],poly2_dataout[59],poly2_dataout[58],poly2_dataout[57],poly2_dataout[56],poly2_dataout[55],poly2_dataout[54],poly2_dataout[53],poly2_dataout[52],poly2_dataout[51],poly2_dataout[50],poly2_dataout[49],poly2_dataout[48]^poly2_dataout[83],poly2_dataout[47],poly2_dataout[46],poly2_dataout[45],poly2_dataout[44],poly2_dataout[43],poly2_dataout[42],poly2_dataout[41],poly2_dataout[40],poly2_dataout[39],poly2_dataout[38],poly2_dataout[37],poly2_dataout[36],poly2_dataout[35],poly2_dataout[34],poly2_dataout[33],poly2_dataout[32],poly2_dataout[31],poly2_dataout[30],poly2_dataout[29]^poly2_dataout[83],poly2_dataout[28],poly2_dataout[27],poly2_dataout[26],poly2_dataout[25],poly2_dataout[24],poly2_dataout[23],poly2_dataout[22],poly2_dataout[21],poly2_dataout[20],poly2_dataout[19],poly2_dataout[18],poly2_dataout[17],poly2_dataout[16],poly2_dataout[15],poly2_dataout[14]^poly2_dataout[83],poly2_dataout[13],poly2_dataout[12],poly2_dataout[11],poly2_dataout[10],poly2_dataout[09],poly2_dataout[08],poly2_dataout[07],poly2_dataout[06],poly2_dataout[05],poly2_dataout[04],poly2_dataout[03],poly2_dataout[02],poly2_dataout[01],poly2_dataout[00],poly2_dataout[83]};
//$display ("for i=%d value of poly2_dataout is %h\n",i,poly2_dataout);
	  end
	  end
	  end
	  end

endmodule 
	 
	 
	
	 
	  
	  
	 
	 
 
// Code your design here
module poly3(clk,rst,addr,write,pushin,wdata,poly3_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output  reg [60:0] poly3_dataout;
 reg [60:0] p3;
 wire [60:0] poly3_dataout_d;
 always @(posedge clk or posedge rst) begin
   if(rst)
    begin
      poly3_dataout<=64'b0;
	end
   else
    begin
         if((write==1'b1) &&(addr==12'h029))
          begin
           poly3_dataout[31:0]=wdata;
          end
         if((write==1'b1)&&(addr==12'h02a))
          begin
           poly3_dataout[60:32]=wdata;
          end

if(pushin) begin	

for (int i=0;i<12;i=i+1) 
	 begin
	   poly3_dataout={  poly3_dataout[59],poly3_dataout[58]^poly3_dataout[60],poly3_dataout[57],poly3_dataout[56],poly3_dataout[55],poly3_dataout[54],poly3_dataout[53],poly3_dataout[52],poly3_dataout[51]^poly3_dataout[60],
	          poly3_dataout[50],poly3_dataout[49],poly3_dataout[48],poly3_dataout[47],poly3_dataout[46]^poly3_dataout[60],poly3_dataout[45],poly3_dataout[44],poly3_dataout[43],poly3_dataout[42],poly3_dataout[41],poly3_dataout[40],
	          poly3_dataout[39],poly3_dataout[38],poly3_dataout[37]^poly3_dataout[60],poly3_dataout[36],poly3_dataout[35],poly3_dataout[34],poly3_dataout[33],poly3_dataout[32]^poly3_dataout[60],poly3_dataout[31],poly3_dataout[30],
	          poly3_dataout[29],poly3_dataout[28],poly3_dataout[27],poly3_dataout[26],poly3_dataout[25],poly3_dataout[24],poly3_dataout[23],poly3_dataout[22],poly3_dataout[21],poly3_dataout[20],
	          poly3_dataout[19],poly3_dataout[18],poly3_dataout[17],poly3_dataout[16],poly3_dataout[15],poly3_dataout[14],poly3_dataout[13],poly3_dataout[12],poly3_dataout[11],poly3_dataout[10],
	          poly3_dataout[09],poly3_dataout[08],poly3_dataout[07],poly3_dataout[06],poly3_dataout[05],poly3_dataout[04],poly3_dataout[03],poly3_dataout[02],poly3_dataout[01],poly3_dataout[00],poly3_dataout[60]};
	  end
//$display ("for value of poly3_dataout is %h\n",poly3_dataout);
	  end
	  end
	  end

endmodule 
	 
	 
	
	 
	  
	  
	 
	 
 
module poly4(clk,rst,addr,write,pushin,wdata,poly4_dataout,poly4_tap);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 output reg  [6:0] poly4_tap;
 input [31:0] wdata;
 output reg [49:0] poly4_dataout;

 reg [49:0] p4;
wire [49:0] poly4_dataout_d;
  
 always @(posedge clk or posedge rst)
 begin
  if(rst)
   begin
   poly4_dataout<=64'b0;
 poly4_tap<=0; 
  end
  else
   begin
     if((write==1'b1) &&(addr==12'h02b))
    begin   
      poly4_dataout[31:0]=wdata;
    end     
  else if((write==1'b1)&&(addr==12'h02c))
    begin
      poly4_dataout[49:32]=wdata;
	end
	if(pushin) begin
	for (int i=0;i<10;i=i+1) 
	 begin
          poly4_dataout={  poly4_dataout[48],poly4_dataout[47],poly4_dataout[46],poly4_dataout[45],poly4_dataout[44],poly4_dataout[43],poly4_dataout[42],poly4_dataout[41],poly4_dataout[40],
	          poly4_dataout[39],poly4_dataout[38],poly4_dataout[37],poly4_dataout[36],poly4_dataout[35]^poly4_dataout[49],poly4_dataout[34],poly4_dataout[33],poly4_dataout[32],poly4_dataout[31],poly4_dataout[30],
	          poly4_dataout[29],poly4_dataout[28],poly4_dataout[27],poly4_dataout[26],poly4_dataout[25],poly4_dataout[24],poly4_dataout[23],poly4_dataout[22],poly4_dataout[21],poly4_dataout[20]^poly4_dataout[49],
	          poly4_dataout[19],poly4_dataout[18],poly4_dataout[17],poly4_dataout[16],poly4_dataout[15]^poly4_dataout[49],poly4_dataout[14],poly4_dataout[13],poly4_dataout[12],poly4_dataout[11],poly4_dataout[10],
	          poly4_dataout[09],poly4_dataout[08],poly4_dataout[07],poly4_dataout[06],poly4_dataout[05]^poly4_dataout[49],poly4_dataout[04]^poly4_dataout[49],poly4_dataout[03],poly4_dataout[02],poly4_dataout[01],poly4_dataout[00],poly4_dataout[49]};
        	poly4_tap[i]=poly4_dataout[0];
	  end
	   end
//$display ("for value of poly4_dataout is %h\n",poly4_dataout);
$display ("for value of poly4_tap is %b\n",poly4_tap);
	  end
	  end

endmodule 
module poly5(clk,rst,addr,write,pushin,wdata,poly5_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output  reg [341:0] poly5_dataout;
 wire [341:0] poly5_dataout_d;

always @(posedge clk or posedge rst)
 begin
if (rst) begin
poly5_dataout<=0;
end
else begin
      if((write==1'b1) &&(addr==12'h033))
          begin
           poly5_dataout[31:0]=wdata;
          end
         if((write==1'b1)&&(addr==12'h034))
          begin
           poly5_dataout[63:32]=wdata;
          end
         if((write==1'b1)&&(addr==12'h035))
          begin
           poly5_dataout[95:64]=wdata;
          end
          if((write==1'b1)&&(addr==12'h036))
          begin
           poly5_dataout[127:96]=wdata;
          end
          if((write==1'b1)&&(addr==12'h037))
          begin
           poly5_dataout[159:128]=wdata;
          end
          if((write==1'b1)&&(addr==12'h038))
          begin
           poly5_dataout[191:160]=wdata;
          end
          if((write==1'b1)&&(addr==12'h039))
          begin
           poly5_dataout[223:192]=wdata;
          end
          if((write==1'b1)&&(addr==12'h03a))
          begin
           poly5_dataout[255:224]=wdata;
          end
      if((write==1'b1)&&(addr==12'h03b))
          begin
           poly5_dataout[287:256]=wdata;
          end
          if((write==1'b1)&&(addr==12'h03c))
          begin
           poly5_dataout[319:288]=wdata;
          end
          if((write==1'b1)&&(addr==12'h03d))
          begin
           poly5_dataout[341:320]=wdata;
          end

if(pushin) begin
	for (int i=0;i<15;i=i+1) 
	 begin
	   poly5_dataout={ poly5_dataout[340],poly5_dataout[339],poly5_dataout[338],poly5_dataout[337],poly5_dataout[336],poly5_dataout[335],poly5_dataout[334],poly5_dataout[333],poly5_dataout[332],poly5_dataout[331],poly5_dataout[330],poly5_dataout[329],poly5_dataout[328],poly5_dataout[327],poly5_dataout[326],poly5_dataout[325],poly5_dataout[324],poly5_dataout[323],poly5_dataout[322],poly5_dataout[321],poly5_dataout[320],poly5_dataout[319],poly5_dataout[318],poly5_dataout[317],poly5_dataout[316],poly5_dataout[315]^poly5_dataout[341],poly5_dataout[314],poly5_dataout[313],poly5_dataout[312],poly5_dataout[311],poly5_dataout[310],poly5_dataout[309]^poly5_dataout[341],poly5_dataout[308],poly5_dataout[307],poly5_dataout[306],poly5_dataout[305],poly5_dataout[304],poly5_dataout[303],poly5_dataout[302],poly5_dataout[301],poly5_dataout[300],poly5_dataout[299],poly5_dataout[298],poly5_dataout[297],poly5_dataout[296],poly5_dataout[295],poly5_dataout[294],poly5_dataout[293],poly5_dataout[292],poly5_dataout[291],poly5_dataout[290],poly5_dataout[289],poly5_dataout[288],poly5_dataout[287],poly5_dataout[286],poly5_dataout[285],poly5_dataout[284],poly5_dataout[283],poly5_dataout[282],poly5_dataout[281],poly5_dataout[280]^poly5_dataout[341],poly5_dataout[279],poly5_dataout[278],poly5_dataout[277],poly5_dataout[276],poly5_dataout[275],poly5_dataout[274],poly5_dataout[273],poly5_dataout[272]^poly5_dataout[341],poly5_dataout[271],poly5_dataout[270],poly5_dataout[269],poly5_dataout[268],poly5_dataout[267],poly5_dataout[266],poly5_dataout[265],poly5_dataout[264],poly5_dataout[263],poly5_dataout[262],poly5_dataout[261],poly5_dataout[260],poly5_dataout[259],poly5_dataout[258],poly5_dataout[257],poly5_dataout[256],poly5_dataout[255],poly5_dataout[254],poly5_dataout[253],poly5_dataout[252],poly5_dataout[251],poly5_dataout[250],poly5_dataout[249],poly5_dataout[248],poly5_dataout[247],poly5_dataout[246],poly5_dataout[245],poly5_dataout[244],poly5_dataout[243],poly5_dataout[242],poly5_dataout[241],poly5_dataout[240],poly5_dataout[239]^poly5_dataout[341],poly5_dataout[238],poly5_dataout[237],poly5_dataout[236],poly5_dataout[235],poly5_dataout[234],poly5_dataout[233],poly5_dataout[232],poly5_dataout[231],poly5_dataout[230],poly5_dataout[229],poly5_dataout[228],poly5_dataout[227],poly5_dataout[226],poly5_dataout[225],poly5_dataout[224],poly5_dataout[223],poly5_dataout[222],poly5_dataout[221],poly5_dataout[220],poly5_dataout[219],poly5_dataout[218],poly5_dataout[217],poly5_dataout[216],poly5_dataout[215],poly5_dataout[214],poly5_dataout[213],poly5_dataout[212],poly5_dataout[211],poly5_dataout[210],poly5_dataout[209],poly5_dataout[208],poly5_dataout[207],poly5_dataout[206],poly5_dataout[205],poly5_dataout[204],poly5_dataout[203],poly5_dataout[202],poly5_dataout[201],poly5_dataout[200],poly5_dataout[199],poly5_dataout[198],poly5_dataout[197],poly5_dataout[196],poly5_dataout[195],poly5_dataout[194],poly5_dataout[193],poly5_dataout[192],poly5_dataout[191],poly5_dataout[190],poly5_dataout[189],poly5_dataout[188],poly5_dataout[187],poly5_dataout[186],poly5_dataout[185],poly5_dataout[184],poly5_dataout[183],poly5_dataout[182],poly5_dataout[181],poly5_dataout[180],poly5_dataout[179],poly5_dataout[178],poly5_dataout[177],poly5_dataout[176],poly5_dataout[175],poly5_dataout[174],poly5_dataout[173],poly5_dataout[172],poly5_dataout[171],poly5_dataout[170],poly5_dataout[169],poly5_dataout[168],poly5_dataout[167],poly5_dataout[166],poly5_dataout[165],poly5_dataout[164],poly5_dataout[163],poly5_dataout[162],poly5_dataout[161],poly5_dataout[160],poly5_dataout[159],poly5_dataout[158],poly5_dataout[157],poly5_dataout[156],poly5_dataout[155],poly5_dataout[154],poly5_dataout[153],poly5_dataout[152],poly5_dataout[151],poly5_dataout[150],poly5_dataout[149],poly5_dataout[148],poly5_dataout[147],poly5_dataout[146],poly5_dataout[145],poly5_dataout[144],poly5_dataout[143],poly5_dataout[142],poly5_dataout[141],poly5_dataout[140],poly5_dataout[139],poly5_dataout[138],poly5_dataout[137],poly5_dataout[136],poly5_dataout[135],poly5_dataout[134],poly5_dataout[133],poly5_dataout[132],poly5_dataout[131],poly5_dataout[130],poly5_dataout[129],poly5_dataout[128],poly5_dataout[127],poly5_dataout[126],poly5_dataout[125],poly5_dataout[124],poly5_dataout[123],poly5_dataout[122],poly5_dataout[121],poly5_dataout[120],poly5_dataout[119],poly5_dataout[118],poly5_dataout[117],poly5_dataout[116],poly5_dataout[115],poly5_dataout[114],poly5_dataout[113],poly5_dataout[112],poly5_dataout[111],poly5_dataout[110],poly5_dataout[109],poly5_dataout[108],poly5_dataout[107],poly5_dataout[106],poly5_dataout[105],poly5_dataout[104],poly5_dataout[103],poly5_dataout[102],poly5_dataout[101],poly5_dataout[100],poly5_dataout[99],poly5_dataout[98],poly5_dataout[97],poly5_dataout[96],poly5_dataout[95],poly5_dataout[94],poly5_dataout[93],poly5_dataout[92],poly5_dataout[91],poly5_dataout[90],poly5_dataout[89],poly5_dataout[88],poly5_dataout[87],poly5_dataout[86],poly5_dataout[85],poly5_dataout[84],poly5_dataout[83],poly5_dataout[82],poly5_dataout[81],poly5_dataout[80],poly5_dataout[79],poly5_dataout[78],poly5_dataout[77],poly5_dataout[76],poly5_dataout[75],poly5_dataout[74],poly5_dataout[73],poly5_dataout[72],poly5_dataout[71],poly5_dataout[70],poly5_dataout[69],poly5_dataout[68],poly5_dataout[67],poly5_dataout[66],poly5_dataout[65],poly5_dataout[64],poly5_dataout[63],poly5_dataout[62],poly5_dataout[61],poly5_dataout[60],poly5_dataout[59],poly5_dataout[58],poly5_dataout[57],poly5_dataout[56],poly5_dataout[55],poly5_dataout[54],poly5_dataout[53],poly5_dataout[52],poly5_dataout[51],poly5_dataout[50],poly5_dataout[49],poly5_dataout[48],poly5_dataout[47],poly5_dataout[46],poly5_dataout[45],poly5_dataout[44],poly5_dataout[43],poly5_dataout[42],poly5_dataout[41],poly5_dataout[40],poly5_dataout[39],poly5_dataout[38],poly5_dataout[37],poly5_dataout[36],poly5_dataout[35],poly5_dataout[34],poly5_dataout[33],poly5_dataout[32],poly5_dataout[31],poly5_dataout[30],poly5_dataout[29],poly5_dataout[28],poly5_dataout[27],poly5_dataout[26],poly5_dataout[25],poly5_dataout[24],poly5_dataout[23],poly5_dataout[22],poly5_dataout[21],poly5_dataout[20],poly5_dataout[19],poly5_dataout[18],poly5_dataout[17],poly5_dataout[16],poly5_dataout[15],poly5_dataout[14],poly5_dataout[13],poly5_dataout[12],poly5_dataout[11],poly5_dataout[10],poly5_dataout[9],poly5_dataout[8],poly5_dataout[7],poly5_dataout[6],poly5_dataout[5],poly5_dataout[4],poly5_dataout[3],poly5_dataout[2],poly5_dataout[1], poly5_dataout[0],poly5_dataout[341]};
	  end
//$display ("for value of poly5_dataout is %h\n",poly5_dataout);
	  end
	  end
	  end
endmodule 


module poly6(clk,rst,addr,write,pushin,wdata,poly6_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [146:0] poly6_dataout;
 reg [146:0] p6 ;
 

always @(posedge clk or posedge rst)
begin
	   if(rst)
	   begin
	   poly6_dataout<=147'b0;
	   end
    else
        begin
      if((write==1'b1) &&(addr==12'h03e))
          begin
          poly6_dataout[31:0]=wdata;
          end
         if((write==1'b1)&&(addr==12'h3f))
          begin
           poly6_dataout[63:32]=wdata;
          end
        if((write==1'b1)&&(addr==12'h040))
          begin
           poly6_dataout[95:64]=wdata;
           end
           if((write==1'b1)&&(addr==12'h041))
          begin
           poly6_dataout[127:96]=wdata;
           end
           if((write==1'b1)&&(addr==12'h042))
          begin
           poly6_dataout[146:128]=wdata;
           end

	if(pushin) begin
	   for (int i=0;i<13;i=i+1) 
	 begin
poly6_dataout={poly6_dataout[145],poly6_dataout[144],poly6_dataout[143],poly6_dataout[142],poly6_dataout[141]^poly6_dataout[146],poly6_dataout[140]^poly6_dataout[146],poly6_dataout[139],poly6_dataout[138],poly6_dataout[137],poly6_dataout[136],poly6_dataout[135],poly6_dataout[134],poly6_dataout[133],poly6_dataout[132],poly6_dataout[131],poly6_dataout[130],poly6_dataout[129],poly6_dataout[128],poly6_dataout[127],poly6_dataout[126],poly6_dataout[125],poly6_dataout[124],poly6_dataout[123],poly6_dataout[122],poly6_dataout[121]^poly6_dataout[146],poly6_dataout[120],poly6_dataout[119],poly6_dataout[118],poly6_dataout[117]^poly6_dataout[146],poly6_dataout[116],poly6_dataout[115],poly6_dataout[114],poly6_dataout[113],poly6_dataout[112],poly6_dataout[111],poly6_dataout[110],poly6_dataout[109],poly6_dataout[108],poly6_dataout[107],poly6_dataout[106],poly6_dataout[105],poly6_dataout[104],poly6_dataout[103],poly6_dataout[102],poly6_dataout[101],poly6_dataout[100],poly6_dataout[99],poly6_dataout[98],poly6_dataout[97],poly6_dataout[96],poly6_dataout[95],poly6_dataout[94],poly6_dataout[93],poly6_dataout[92],poly6_dataout[91],poly6_dataout[90],poly6_dataout[89],poly6_dataout[88],poly6_dataout[87],poly6_dataout[86],poly6_dataout[85],poly6_dataout[84],poly6_dataout[83],poly6_dataout[82],poly6_dataout[81],poly6_dataout[80],poly6_dataout[79],poly6_dataout[78],poly6_dataout[77],poly6_dataout[76],poly6_dataout[75],poly6_dataout[74],poly6_dataout[73],poly6_dataout[72],poly6_dataout[71],poly6_dataout[70],poly6_dataout[69],poly6_dataout[68],poly6_dataout[67],poly6_dataout[66],poly6_dataout[65],poly6_dataout[64],poly6_dataout[63],poly6_dataout[62],poly6_dataout[61],poly6_dataout[60],poly6_dataout[59],poly6_dataout[58],poly6_dataout[57],poly6_dataout[56],poly6_dataout[55],poly6_dataout[54],poly6_dataout[53],poly6_dataout[52],poly6_dataout[51],poly6_dataout[50],poly6_dataout[49]^poly6_dataout[146],poly6_dataout[48],poly6_dataout[47],poly6_dataout[46],poly6_dataout[45],poly6_dataout[44],poly6_dataout[43],poly6_dataout[42],poly6_dataout[41],poly6_dataout[40],poly6_dataout[39],poly6_dataout[38],poly6_dataout[37],poly6_dataout[36],poly6_dataout[35],poly6_dataout[34],poly6_dataout[33],poly6_dataout[32],poly6_dataout[31],poly6_dataout[30],poly6_dataout[29],poly6_dataout[28],poly6_dataout[27],poly6_dataout[26],poly6_dataout[25],poly6_dataout[24],poly6_dataout[23],poly6_dataout[22],poly6_dataout[21],poly6_dataout[20],poly6_dataout[19],poly6_dataout[18],poly6_dataout[17],poly6_dataout[16],poly6_dataout[15],poly6_dataout[14],poly6_dataout[13],poly6_dataout[12],poly6_dataout[11],poly6_dataout[10],poly6_dataout[9],poly6_dataout[8],poly6_dataout[7],poly6_dataout[6],poly6_dataout[5],poly6_dataout[4],poly6_dataout[3],poly6_dataout[2],poly6_dataout[1],poly6_dataout[0], poly6_dataout[146]};
	  end
//$display ("for value of poly6_dataout is %h\n",poly6_dataout);
	  end
	  end
	  end

endmodule 


module poly7(clk,rst,addr,write,pushin,wdata,poly7_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [346:0] poly7_dataout;
 reg [346:0] p7 ;
 wire  [346:0] poly7_dataout_d;

always @(posedge clk or posedge rst)
begin
	   if(rst)
	   begin
	   poly7_dataout<=347'b0;
	   end
    else
        begin
     if((write==1'b1) &&(addr==12'h044))
          begin
          poly7_dataout[31:0]=wdata;
          end
         if((write==1'b1)&&(addr==12'h045))
          begin
           poly7_dataout[63:32]=wdata;
          end
        if((write==1'b1)&&(addr==12'h046))
          begin
           poly7_dataout[95:64]=wdata;
           end
           if((write==1'b1)&&(addr==12'h047))
          begin
           poly7_dataout[127:96]=wdata;
           end
           if((write==1'b1)&&(addr==12'h048))
          begin
           poly7_dataout[159:128]=wdata;
           end
              if((write==1'b1)&&(addr==12'h049))
          begin
           poly7_dataout[191:160]=wdata;
           end
              if((write==1'b1)&&(addr==12'h04a))
          begin
           poly7_dataout[223:192]=wdata;
           end
              if((write==1'b1)&&(addr==12'h04b))
          begin
           poly7_dataout[255:224]=wdata;
           end
              if((write==1'b1)&&(addr==12'h04c))
          begin
           poly7_dataout[287:256]=wdata;
           end
              if((write==1'b1)&&(addr==12'h04d))
          begin
           poly7_dataout[319:288]=wdata;
           end
              if((write==1'b1)&&(addr==12'h04e))
          begin
           poly7_dataout[346:320]=wdata;
           end
if(pushin) begin
  for (int i=0;i<18;i=i+1) 
     begin
    poly7_dataout={ poly7_dataout[345],poly7_dataout[344],poly7_dataout[343],poly7_dataout[342],poly7_dataout[341],poly7_dataout[340],poly7_dataout[339],poly7_dataout[338],poly7_dataout[337],poly7_dataout[336],poly7_dataout[335],poly7_dataout[334],poly7_dataout[333],poly7_dataout[332],poly7_dataout[331],poly7_dataout[330],poly7_dataout[329],poly7_dataout[328],poly7_dataout[327],poly7_dataout[326],poly7_dataout[325],poly7_dataout[324],poly7_dataout[323],poly7_dataout[322],poly7_dataout[321],poly7_dataout[320],poly7_dataout[319],poly7_dataout[318],poly7_dataout[317],poly7_dataout[316],poly7_dataout[315],poly7_dataout[314],poly7_dataout[313],poly7_dataout[312],poly7_dataout[311],poly7_dataout[310],poly7_dataout[309],poly7_dataout[308],poly7_dataout[307],poly7_dataout[306],poly7_dataout[305],poly7_dataout[304],poly7_dataout[303],poly7_dataout[302],poly7_dataout[301],poly7_dataout[300],poly7_dataout[299],poly7_dataout[298],poly7_dataout[297],poly7_dataout[296],poly7_dataout[295],poly7_dataout[294],poly7_dataout[293],poly7_dataout[292],poly7_dataout[291],poly7_dataout[290],poly7_dataout[289],poly7_dataout[288],poly7_dataout[287],poly7_dataout[286],poly7_dataout[285],poly7_dataout[284],poly7_dataout[283],poly7_dataout[282],poly7_dataout[281],poly7_dataout[280],poly7_dataout[279],poly7_dataout[278],poly7_dataout[277],poly7_dataout[276],poly7_dataout[275],poly7_dataout[274],poly7_dataout[273],poly7_dataout[272],poly7_dataout[271],poly7_dataout[270],poly7_dataout[269],poly7_dataout[268],poly7_dataout[267],poly7_dataout[266],poly7_dataout[265],poly7_dataout[264],poly7_dataout[263],poly7_dataout[262],poly7_dataout[261],poly7_dataout[260],poly7_dataout[259],poly7_dataout[258],poly7_dataout[257],poly7_dataout[256],poly7_dataout[255],poly7_dataout[254],poly7_dataout[253],poly7_dataout[252],poly7_dataout[251],poly7_dataout[250],poly7_dataout[249],poly7_dataout[248],poly7_dataout[247],poly7_dataout[246],poly7_dataout[245],poly7_dataout[244],poly7_dataout[243],poly7_dataout[242],poly7_dataout[241],poly7_dataout[240],poly7_dataout[239],poly7_dataout[238],poly7_dataout[237],poly7_dataout[236],poly7_dataout[235]^poly7_dataout[346],poly7_dataout[234],poly7_dataout[233],poly7_dataout[232],poly7_dataout[231],poly7_dataout[230],poly7_dataout[229],poly7_dataout[228],poly7_dataout[227],poly7_dataout[226],poly7_dataout[225],poly7_dataout[224],poly7_dataout[223],poly7_dataout[222],poly7_dataout[221],poly7_dataout[220],poly7_dataout[219],poly7_dataout[218],poly7_dataout[217],poly7_dataout[216],poly7_dataout[215],poly7_dataout[214],poly7_dataout[213],poly7_dataout[212],poly7_dataout[211],poly7_dataout[210],poly7_dataout[209],poly7_dataout[208]^poly7_dataout[346],poly7_dataout[207],poly7_dataout[206],poly7_dataout[205],poly7_dataout[204],poly7_dataout[203],poly7_dataout[202],poly7_dataout[201],poly7_dataout[200],poly7_dataout[199],poly7_dataout[198],poly7_dataout[197],poly7_dataout[196],poly7_dataout[195],poly7_dataout[194],poly7_dataout[193],poly7_dataout[192],poly7_dataout[191],poly7_dataout[190],poly7_dataout[189],poly7_dataout[188],poly7_dataout[187],poly7_dataout[186],poly7_dataout[185],poly7_dataout[184],poly7_dataout[183],poly7_dataout[182],poly7_dataout[181],poly7_dataout[180],poly7_dataout[179],poly7_dataout[178],poly7_dataout[177],poly7_dataout[176],poly7_dataout[175],poly7_dataout[174],poly7_dataout[173],poly7_dataout[172],poly7_dataout[171],poly7_dataout[170],poly7_dataout[169],poly7_dataout[168],poly7_dataout[167],poly7_dataout[166],poly7_dataout[165],poly7_dataout[164],poly7_dataout[163],poly7_dataout[162],poly7_dataout[161]^poly7_dataout[346],poly7_dataout[160],poly7_dataout[159],poly7_dataout[158],poly7_dataout[157],poly7_dataout[156],poly7_dataout[155],poly7_dataout[154],poly7_dataout[153],poly7_dataout[152],poly7_dataout[151],poly7_dataout[150],poly7_dataout[149],poly7_dataout[148],poly7_dataout[147],poly7_dataout[146],poly7_dataout[145],poly7_dataout[144],poly7_dataout[143],poly7_dataout[142],poly7_dataout[141],poly7_dataout[140],poly7_dataout[139],poly7_dataout[138],poly7_dataout[137],poly7_dataout[136],poly7_dataout[135],poly7_dataout[134],poly7_dataout[133],poly7_dataout[132],poly7_dataout[131],poly7_dataout[130],poly7_dataout[129],poly7_dataout[128],poly7_dataout[127],poly7_dataout[126],poly7_dataout[125],poly7_dataout[124],poly7_dataout[123],poly7_dataout[122],poly7_dataout[121],poly7_dataout[120],poly7_dataout[119],poly7_dataout[118],poly7_dataout[117],poly7_dataout[116],poly7_dataout[115],poly7_dataout[114],poly7_dataout[113],poly7_dataout[112],poly7_dataout[111],poly7_dataout[110],poly7_dataout[109],poly7_dataout[108],poly7_dataout[107],poly7_dataout[106],poly7_dataout[105],poly7_dataout[104],poly7_dataout[103],poly7_dataout[102],poly7_dataout[101],poly7_dataout[100],poly7_dataout[99],poly7_dataout[98],poly7_dataout[97],poly7_dataout[96],poly7_dataout[95],poly7_dataout[94],poly7_dataout[93],poly7_dataout[92],poly7_dataout[91],poly7_dataout[90],poly7_dataout[89],poly7_dataout[88],poly7_dataout[87],poly7_dataout[86],poly7_dataout[85],poly7_dataout[84],poly7_dataout[83],poly7_dataout[82],poly7_dataout[81],poly7_dataout[80],poly7_dataout[79],poly7_dataout[78],poly7_dataout[77],poly7_dataout[76],poly7_dataout[75],poly7_dataout[74],poly7_dataout[73],poly7_dataout[72],poly7_dataout[71],poly7_dataout[70],poly7_dataout[69],poly7_dataout[68],poly7_dataout[67],poly7_dataout[66],poly7_dataout[65],poly7_dataout[64],poly7_dataout[63]^poly7_dataout[346],poly7_dataout[62],poly7_dataout[61],poly7_dataout[60],poly7_dataout[59],poly7_dataout[58],poly7_dataout[57],poly7_dataout[56],poly7_dataout[55],poly7_dataout[54],poly7_dataout[53],poly7_dataout[52],poly7_dataout[51],poly7_dataout[50],poly7_dataout[49],poly7_dataout[48],poly7_dataout[47],poly7_dataout[46],poly7_dataout[45],poly7_dataout[44],poly7_dataout[43],poly7_dataout[42],poly7_dataout[41],poly7_dataout[40],poly7_dataout[39],poly7_dataout[38],poly7_dataout[37],poly7_dataout[36],poly7_dataout[35],poly7_dataout[34],poly7_dataout[33],poly7_dataout[32],poly7_dataout[31],poly7_dataout[30]^poly7_dataout[346],poly7_dataout[29],poly7_dataout[28],poly7_dataout[27],poly7_dataout[26],poly7_dataout[25],poly7_dataout[24],poly7_dataout[23],poly7_dataout[22],poly7_dataout[21],poly7_dataout[20],poly7_dataout[19],poly7_dataout[18],poly7_dataout[17],poly7_dataout[16],poly7_dataout[15],poly7_dataout[14],poly7_dataout[13],poly7_dataout[12],poly7_dataout[11],poly7_dataout[10],poly7_dataout[9],poly7_dataout[8],poly7_dataout[7],poly7_dataout[6],poly7_dataout[5],poly7_dataout[4],poly7_dataout[3],poly7_dataout[2],poly7_dataout[1],poly7_dataout[0],poly7_dataout[346]};
	  end
//$display ("for value of poly7_dataout is %h\n",poly7_dataout);
	  end
	  end
	  end
endmodule 

module poly8(clk,rst,addr,write,pushin,wdata,poly8_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output reg [197:0] poly8_dataout;
 

always @(posedge clk or posedge rst)
begin
      if(rst)
      begin
      poly8_dataout<=198'b0;
      end
    else
        begin
      if((write==1'b1) &&(addr==12'h056))
          begin
          poly8_dataout[31:0]=wdata;
          end
         if((write==1'b1)&&(addr==12'h057))
          begin
           poly8_dataout[63:32]=wdata;
          end
        if((write==1'b1)&&(addr==12'h058))
          begin
           poly8_dataout[95:64]=wdata;
           end
           if((write==1'b1)&&(addr==12'h059))
          begin
           poly8_dataout[127:96]=wdata;
           end
           if((write==1'b1)&&(addr==12'h05a))
          begin
           poly8_dataout[159:128]=wdata;
           end
              if((write==1'b1)&&(addr==12'h05b))
          begin
           poly8_dataout[191:160]=wdata;
           end
              if((write==1'b1)&&(addr==12'h05c))
          begin
           poly8_dataout[197:192]=wdata;
           end

	if(pushin) begin
            for (int i=0;i<12;i=i+1) 
            begin
	   poly8_dataout={poly8_dataout[196],poly8_dataout[195],poly8_dataout[194],poly8_dataout[193],poly8_dataout[192],poly8_dataout[191],poly8_dataout[190],poly8_dataout[189],poly8_dataout[188],poly8_dataout[187],poly8_dataout[186],poly8_dataout[185],poly8_dataout[184],poly8_dataout[183],poly8_dataout[182],poly8_dataout[181],poly8_dataout[180],poly8_dataout[179],poly8_dataout[178],poly8_dataout[177],poly8_dataout[176],poly8_dataout[175],poly8_dataout[174],poly8_dataout[173],poly8_dataout[172],poly8_dataout[171],poly8_dataout[170],poly8_dataout[169],poly8_dataout[168],poly8_dataout[167],poly8_dataout[166],poly8_dataout[165],poly8_dataout[164],poly8_dataout[163],poly8_dataout[162],poly8_dataout[161],poly8_dataout[160],poly8_dataout[159],poly8_dataout[158],poly8_dataout[157],poly8_dataout[156],poly8_dataout[155],poly8_dataout[154],poly8_dataout[153],poly8_dataout[152],poly8_dataout[151],poly8_dataout[150],poly8_dataout[149],poly8_dataout[148],poly8_dataout[147],poly8_dataout[146],poly8_dataout[145],poly8_dataout[144],poly8_dataout[143],poly8_dataout[142]^poly8_dataout[197],poly8_dataout[141],poly8_dataout[140],poly8_dataout[139],poly8_dataout[138],poly8_dataout[137],poly8_dataout[136],poly8_dataout[135],poly8_dataout[134],poly8_dataout[133],poly8_dataout[132],poly8_dataout[131],poly8_dataout[130],poly8_dataout[129],poly8_dataout[128],poly8_dataout[127],poly8_dataout[126],poly8_dataout[125],poly8_dataout[124],poly8_dataout[123],poly8_dataout[122],poly8_dataout[121],poly8_dataout[120],poly8_dataout[119],poly8_dataout[118],poly8_dataout[117],poly8_dataout[116],poly8_dataout[115],poly8_dataout[114],poly8_dataout[113],poly8_dataout[112],poly8_dataout[111],poly8_dataout[110],poly8_dataout[109],poly8_dataout[108],poly8_dataout[107]^poly8_dataout[197],poly8_dataout[106]^poly8_dataout[197],poly8_dataout[105],poly8_dataout[104],poly8_dataout[103],poly8_dataout[102]^poly8_dataout[197],poly8_dataout[101],poly8_dataout[100],poly8_dataout[99],poly8_dataout[98],poly8_dataout[97],poly8_dataout[96],poly8_dataout[95],poly8_dataout[94],poly8_dataout[93],poly8_dataout[92],poly8_dataout[91],poly8_dataout[90],poly8_dataout[89],poly8_dataout[88],poly8_dataout[87],poly8_dataout[86],poly8_dataout[85],poly8_dataout[84],poly8_dataout[83],poly8_dataout[82],poly8_dataout[81]^poly8_dataout[197],poly8_dataout[80],poly8_dataout[79],poly8_dataout[78],poly8_dataout[77],poly8_dataout[76],poly8_dataout[75],poly8_dataout[74],poly8_dataout[73],poly8_dataout[72],poly8_dataout[71],poly8_dataout[70],poly8_dataout[69],poly8_dataout[68],poly8_dataout[67],poly8_dataout[66],poly8_dataout[65],poly8_dataout[64],poly8_dataout[63],poly8_dataout[62],poly8_dataout[61],poly8_dataout[60],poly8_dataout[59],poly8_dataout[58],poly8_dataout[57],poly8_dataout[56],poly8_dataout[55],poly8_dataout[54],poly8_dataout[53],poly8_dataout[52],poly8_dataout[51],poly8_dataout[50],poly8_dataout[49],poly8_dataout[48],poly8_dataout[47],poly8_dataout[46],poly8_dataout[45],poly8_dataout[44],poly8_dataout[43],poly8_dataout[42],poly8_dataout[41],poly8_dataout[40],poly8_dataout[39],poly8_dataout[38],poly8_dataout[37],poly8_dataout[36],poly8_dataout[35],poly8_dataout[34],poly8_dataout[33],poly8_dataout[32],poly8_dataout[31],poly8_dataout[30],poly8_dataout[29],poly8_dataout[28],poly8_dataout[27],poly8_dataout[26],poly8_dataout[25],poly8_dataout[24],poly8_dataout[23],poly8_dataout[22],poly8_dataout[21],poly8_dataout[20],poly8_dataout[19],poly8_dataout[18],poly8_dataout[17],poly8_dataout[16],poly8_dataout[15],poly8_dataout[14],poly8_dataout[13],poly8_dataout[12],poly8_dataout[11],poly8_dataout[10],poly8_dataout[9],poly8_dataout[8],poly8_dataout[7],poly8_dataout[6],poly8_dataout[5],poly8_dataout[4],poly8_dataout[3],poly8_dataout[2],poly8_dataout[1],poly8_dataout[0],poly8_dataout[197]};
	  end
//$display ("for value of poly8_dataout is %h\n",poly8_dataout);
	  end
	  end
	  end
endmodule 
module poly9(clk,rst,addr,write,pushin,wdata,poly9_dataout);
 input clk;
 input rst;
 input [11:0] addr;
 input write,pushin;
 input [31:0] wdata;
 output  reg [42:0] poly9_dataout;

always @(posedge clk or posedge rst)
    begin
          if(rst)
          begin
          poly9_dataout<=43'b0;
          end
    else
        begin
      if((write==1'b1) &&(addr==12'h061))
          begin
          poly9_dataout[31:0]=wdata;
          end
         if((write==1'b1)&&(addr==12'h062))
          begin
           poly9_dataout[42:32]=wdata;
          end
	if(pushin) begin
          for (int i=0;i<18;i=i+1) 
	 begin
	  poly9_dataout ={poly9_dataout[41],poly9_dataout[40],poly9_dataout[39],poly9_dataout[38],poly9_dataout[37],poly9_dataout[36],poly9_dataout[35],poly9_dataout[34]^poly9_dataout[42],poly9_dataout[33],poly9_dataout[32],poly9_dataout[31]^poly9_dataout[42],poly9_dataout[30],poly9_dataout[29]^poly9_dataout[42],poly9_dataout[28],poly9_dataout[27],poly9_dataout[26],poly9_dataout[25],poly9_dataout[24]^poly9_dataout[42],poly9_dataout[23],poly9_dataout[22],poly9_dataout[21],poly9_dataout[20],poly9_dataout[19],poly9_dataout[18],poly9_dataout[17],poly9_dataout[16],poly9_dataout[15],poly9_dataout[14],poly9_dataout[13],poly9_dataout[12],poly9_dataout[11],poly9_dataout[10],poly9_dataout[9],poly9_dataout[8],poly9_dataout[7]^poly9_dataout[42],poly9_dataout[6],poly9_dataout[5],poly9_dataout[4],poly9_dataout[3],poly9_dataout[2],poly9_dataout[1],poly9_dataout[0],poly9_dataout[42]};      
        end
 //$display ("for value of poly9_dataout is %h\n",poly9_dataout);
        end
        end
        end
	 

endmodule 
module decoder_plfsr(clk,rst,pushin,pselect,pdata0,pdata1,pdata2,pdata3,pdata4,pdata5,pdata6,pdata7,pdata8,pdata9,pdata10,pdata11,pdata12,pdata13,pdata14,pdata15,pdata_out,pushout);
input [4:0] pselect;
input [63:0] pdata0;
input [55:0] pdata1;
input [83:0] pdata2;
input [60:0] pdata3;
input [49:0] pdata4;
input [341:0] pdata5;
input [146:0] pdata6;
input [346:0] pdata7;
input [197:0] pdata8;
input [42:0] pdata9;
input [126:0] pdata10;
input [300:0] pdata11;
input [194:0] pdata12;
input [59:0] pdata13;
input [33:0] pdata14;
input [85:0] pdata15;
input clk,rst;
input pushin;
output  reg pushout;
output reg [15:0] pdata_out;
 reg [15:0] pdata_out_d;
always @(*)
begin

case(pselect)
5'd0:pdata_out_d={pdata4[23],pdata10[25],pdata14[12],pdata8[124],pdata9[30],pdata2[35],pdata0[1],pdata15[44],pdata1[20]
        ,pdata3[52],pdata6[14],pdata13[35],pdata11[109],pdata5[207],pdata7[61],pdata12[39]};
5'd1:pdata_out_d={pdata12[137],pdata13[20],pdata1[14],pdata7[177],pdata9[27],pdata0[40],pdata14[14],pdata15[79]
        ,pdata11[18],pdata10[71],pdata5[223],pdata3[29],pdata2[44],pdata8[4],pdata6[36],pdata4[41]};
5'd2:pdata_out_d={pdata8[174],pdata1[50],pdata11[177],pdata3[49],pdata9[28],pdata10[28],pdata14[25],pdata15[31]
        ,pdata2[3],pdata6[22],pdata5[291],pdata0[22],pdata13[27],pdata12[42],pdata7[102],pdata4[16]};
5'd3:pdata_out_d={pdata5[27],pdata10[98],pdata11[232],pdata9[11],pdata4[49],pdata6[54],pdata0[21],pdata2[52],pdata1[37]
        ,pdata15[54],pdata12[31],pdata7[175],pdata8[73],pdata14[22],pdata13[16],pdata3[18]};
5'd4:pdata_out_d={pdata10[15],pdata4[8],pdata6[98],pdata15[32],pdata0[59],pdata11[69],pdata8[33],pdata1[25],pdata9[0]
        ,pdata12[61],pdata13[48],pdata7[265],pdata3[46],pdata14[23],pdata5[94],pdata2[82]};
5'd5:pdata_out_d={pdata2[63],pdata12[35],pdata7[28],pdata11[295],pdata5[45],pdata1[11],pdata13[28],pdata8[137],pdata10[120]
        ,pdata3[6],pdata4[25],pdata14[11],pdata6[68],pdata15[55],pdata9[34],pdata0[63]};
5'd6:pdata_out_d={pdata1[38],pdata12[40],pdata10[115],pdata6[103],pdata3[27],pdata8[92],pdata15[68],pdata2[33],pdata13[4]
        ,pdata11[231],pdata14[16],pdata4[39],pdata5[153],pdata7[308],pdata0[30],pdata9[42]};
5'd7:pdata_out_d={pdata1[48],pdata10[19],pdata4[1],pdata6[138],pdata7[144],pdata3[54],pdata13[2],pdata5[132],pdata11[94]
        ,pdata14[28],pdata0[62],pdata15[23],pdata8[21],pdata9[24],pdata12[33],pdata2[47]};
5'd8:pdata_out_d={pdata12[12],pdata7[11],pdata0[38],pdata5[44],pdata3[10],pdata14[2],pdata13[11],pdata10[90],pdata6[95]
                ,pdata1[32],pdata2[38],pdata15[39],pdata9[15],pdata8[171],pdata4[36],pdata11[259]};
5'd9:pdata_out_d={pdata4[18],pdata5[305],pdata9[35],pdata1[9],pdata12[58],pdata7[26],pdata14[26],pdata13[39],pdata8[145]
        ,pdata0[53],pdata2[5],pdata11[144],pdata15[73],pdata6[122],pdata10[52],pdata3[34]};
5'd10:pdata_out_d={pdata10[83],pdata15[21],pdata12[179],pdata3[48],pdata9[40],pdata1[53],pdata14[27],pdata2[4],pdata4[3]
        ,pdata13[0],pdata7[32],pdata0[60],pdata11[98],pdata6[53],pdata8[7],pdata5[257]};
5'd11:pdata_out_d={pdata12[85],pdata2[22],pdata3[26],pdata11[113],pdata1[22],pdata9[25],pdata14[4],pdata10[16],pdata5[80]
        ,pdata15[8],pdata13[53],pdata4[14],pdata7[189],pdata6[115],pdata0[24],pdata8[0]};
5'd12:pdata_out_d={pdata1[19],pdata8[54],pdata7[194],pdata13[8],pdata11[71],pdata12[92],pdata5[225],pdata0[54],pdata10[5]
        ,pdata3[8],pdata15[29],pdata2[41],pdata6[79],pdata4[7],pdata9[1],pdata14[20]};
5'd13:pdata_out_d={pdata2[27],pdata13[22],pdata8[94],pdata11[233],pdata7[209],pdata15[63],pdata1[16],pdata0[33],pdata5[151]
        ,pdata10[33],pdata14[18],pdata6[21],pdata3[0],pdata4[26],pdata9[14],pdata12[107]};
5'd14:pdata_out_d={pdata14[9],pdata2[49],pdata11[243],pdata15[6],pdata7[16],pdata8[27],pdata12[126],pdata13[52],pdata0[19]
        ,pdata1[34],pdata9[38],pdata4[21],pdata5[102],pdata10[4],pdata3[11],pdata6[73]};
5'd15:pdata_out_d={pdata3[4],pdata11[277],pdata14[19],pdata1[13],pdata12[13],pdata5[93],pdata9[21],pdata2[67],pdata13[6]
        ,pdata7[166],pdata6[46],pdata8[123],pdata0[37],pdata4[45],pdata15[53],pdata10[39]};
5'd16:pdata_out_d={pdata8[192],pdata4[42],pdata10[101],pdata7[133],pdata6[125],pdata15[85],pdata12[46],pdata3[9]
        ,pdata2[13],pdata13[41],pdata11[138],pdata5[79],pdata1[12],pdata0[0],pdata9[39],pdata14[0]};
5'd17:pdata_out_d={pdata12[37],pdata9[7],pdata10[53],pdata5[76],pdata0[9],pdata6[35],pdata2[79],pdata7[152],pdata14[29]
        ,pdata13[12],pdata8[52],pdata15[14],pdata11[261],pdata1[10],pdata4[34],pdata3[7]};
5'd18:pdata_out_d={pdata0[48],pdata4[6],pdata9[5],pdata7[70],pdata8[106],pdata5[260],pdata2[78],pdata6[109],pdata13[14]
        ,pdata11[127],pdata3[24],pdata10[118],pdata1[51],pdata12[7],pdata15[19],pdata14[5]};
5'd19:pdata_out_d={pdata6[48],pdata4[33],pdata1[45],pdata13[31],pdata7[296],pdata0[39],pdata15[37],pdata2[25],pdata8[105]
        ,pdata3[59],pdata5[126],pdata9[20],pdata12[189],pdata11[254],pdata10[88],pdata14[33]};
5'd20:pdata_out_d={pdata11[54],pdata4[28],pdata1[44],pdata10[96],pdata3[39],pdata13[44],pdata9[2],pdata14[8],pdata0[47]
        ,pdata6[34],pdata15[75],pdata12[172],pdata2[75],pdata7[298],pdata5[235],pdata8[152]};
5'd21:pdata_out_d={pdata12[9],pdata7[118],pdata14[3],pdata1[30],pdata9[31],pdata3[2],pdata10[70],pdata2[8],pdata6[44]
        ,pdata13[55],pdata5[5],pdata4[5],pdata15[58],pdata11[257],pdata0[17],pdata8[43]};
5'd22:pdata_out_d={pdata2[28],pdata13[36],pdata6[64],pdata3[1],pdata14[32],pdata9[18],pdata4[4],pdata12[43],pdata0[45]
        ,pdata5[254],pdata10[0],pdata1[52],pdata7[304],pdata11[157],pdata15[26],pdata8[181]};
5'd23:pdata_out_d={pdata9[17],pdata14[13],pdata6[25],pdata8[141],pdata12[190],pdata3[44],pdata4[37],pdata5[246],pdata11[116]
        ,pdata13[18],pdata7[112],pdata2[2],pdata1[40],pdata10[82],pdata15[49],pdata0[58]};
5'd24:pdata_out_d={pdata1[55],pdata6[97],pdata12[139],pdata4[48],pdata15[27],pdata11[128],pdata2[23],pdata14[31]
        ,pdata9[26],pdata3[41],pdata0[51],pdata13[51],pdata8[65],pdata10[104],pdata5[290],pdata7[30]};
5'd25:pdata_out_d={pdata8[23],pdata0[49],pdata6[19],pdata2[73],pdata3[22],pdata5[139],pdata9[3],pdata11[272],pdata14[15]
        ,pdata4[19],pdata1[39],pdata15[13],pdata12[66],pdata10[26],pdata7[103],pdata13[54]};
5'd26:pdata_out_d={pdata8[178],pdata3[17],pdata9[29],pdata5[295],pdata15[18],pdata11[100],pdata6[131],pdata10[32]
        ,pdata1[35],pdata13[7],pdata14[6],pdata2[9],pdata4[9],pdata7[252],pdata0[35],pdata12[47]};
5'd27:pdata_out_d={pdata1[8],pdata15[17],pdata8[46],pdata0[57],pdata12[174],pdata4[24],pdata7[339],pdata9[33],pdata10[11]
        ,pdata6[5],pdata5[63],pdata2[17],pdata14[21],pdata11[173],pdata13[15],pdata3[28]};
5'd28:pdata_out_d={pdata11[104],pdata7[137],pdata6[13],pdata9[13],pdata13[42],pdata8[112],pdata3[15],pdata12[65]
        ,pdata10[74],pdata0[34],pdata4[22],pdata14[10],pdata2[15],pdata1[7],pdata5[111],pdata15[52]};
5'd29:pdata_out_d={pdata1[27],pdata3[50],pdata10[44],pdata4[17],pdata9[19],pdata12[64],pdata2[51],pdata6[114],pdata14[24]
        ,pdata13[25],pdata8[101],pdata0[8],pdata7[195],pdata15[59],pdata11[278],pdata5[326]};
5'd30:pdata_out_d={pdata5[322],pdata7[286],pdata11[64],pdata8[111],pdata0[10],pdata3[47],pdata13[56],pdata12[132]
        ,pdata1[24],pdata6[142],pdata4[31],pdata9[10],pdata14[7],pdata2[70],pdata15[11],pdata10[108]};
5'd31:pdata_out_d={pdata12[82],pdata10[75],pdata0[27],pdata15[16],pdata1[17],pdata13[24],pdata11[110],pdata9[6],pdata14[30]
        ,pdata6[112],pdata5[232],pdata8[134],pdata3[57],pdata4[46],pdata7[150],pdata2[60]};
        endcase
        end
always @(posedge clk or posedge rst) begin
if (rst)
begin
 pdata_out<=0;
 pushout<=0;
end
else begin
pdata_out<=pdata_out_d;

if (pushin)
begin
pushout<=1;
end
else
begin
pushout<=0;
end
end
end
        endmodule
module decoder_entrophy(clk,rst,pushin,dselect,data,entrophy,first_ivalue,pushout);
input [4:0] dselect;
input [7:0] data;
input [31:0] entrophy;
input clk,rst,pushin;
reg [4:0] dummy_s;
output reg [31:0] first_ivalue;
output reg pushout;
reg [31:0] first_ivalue_d;

always @(*)
begin
case (dselect)
  5'd0:first_ivalue_d={entrophy[24],entrophy[1],data[0],entrophy[23],entrophy[5],data[1],entrophy[12],data[2],data[6],data[3],entrophy[0]
        ,data[4],entrophy[30],entrophy[20],entrophy[16],entrophy[27],data[7],entrophy[28],entrophy[25],entrophy[14],entrophy[18],entrophy[22]
        ,entrophy[21],entrophy[26],entrophy[8],data[5],entrophy[10],entrophy[7],entrophy[11],entrophy[13],entrophy[6],entrophy[4]};

5'd1:first_ivalue_d={entrophy[23],data[6],entrophy[15],entrophy[21],entrophy[25],entrophy[11],data[1],entrophy[14],entrophy[7],data[7],entrophy[1],data[3]
        ,entrophy[0],entrophy[4],data[4],entrophy[19],entrophy[3],entrophy[13],entrophy[20],entrophy[6],data[2],entrophy[30],entrophy[18],entrophy[26]
        ,entrophy[10],entrophy[12],data[0],entrophy[29],data[5],entrophy[2],entrophy[28],entrophy[8]};

5'd2:first_ivalue_d={entrophy[20],entrophy[1],data[2],entrophy[19],data[1],entrophy[3],entrophy[25],entrophy[24],entrophy[28],entrophy[21],entrophy[14],entrophy[0]
        ,entrophy[9],entrophy[13],entrophy[16],entrophy[8],entrophy[10],data[7],entrophy[31],entrophy[7],entrophy[26],entrophy[5],entrophy[11],entrophy[18]
        ,entrophy[6],data[3],entrophy[22],entrophy[4],data[5],data[0],data[6],data[4]};

5'd3:first_ivalue_d={entrophy[17],entrophy[29],data[4],entrophy[16],data[3],entrophy[11],entrophy[9],entrophy[18],data[6],data[1],entrophy[30]
        ,entrophy[4],entrophy[5],entrophy[24],data[7],entrophy[27],entrophy[31],entrophy[1],entrophy[20],entrophy[15],data[5],entrophy[22],entrophy[13]
        ,data[2],entrophy[10],entrophy[3],entrophy[26],entrophy[2],entrophy[23],data[0],entrophy[6],entrophy[28]};
5'd4:first_ivalue_d={entrophy[16],entrophy[1],entrophy[12],data[1],entrophy[3],entrophy[2],data[4],entrophy[13],entrophy[8],data[0],entrophy[21],entrophy[18]
        ,entrophy[10],data[2],entrophy[22],entrophy[15],entrophy[17],entrophy[0],entrophy[31],entrophy[11],entrophy[25],entrophy[6],data[6],entrophy[9]
        ,data[5],data[3],entrophy[28],entrophy[23],data[7],entrophy[14],entrophy[4],entrophy[26]};

5'd5:first_ivalue_d={entrophy[11],entrophy[17],entrophy[2],entrophy[7],entrophy[21],entrophy[4],entrophy[24],entrophy[10],data[7],entrophy[19],entrophy[12],data[5]
        ,entrophy[30],data[1],entrophy[9],entrophy[3],entrophy[18],entrophy[5],data[2],data[3],entrophy[31],entrophy[23],data[4]
        ,entrophy[14],entrophy[25],entrophy[26],data[6],entrophy[0],entrophy[1],entrophy[22],entrophy[28],data[0]};

5'd6:first_ivalue_d={entrophy[22],entrophy[13],data[6],entrophy[19],entrophy[27],entrophy[6],data[0],data[7],data[1],data[2],entrophy[10]
        ,entrophy[8],entrophy[0],entrophy[3],entrophy[29],entrophy[20],entrophy[24],data[3],entrophy[14],entrophy[15],entrophy[31],entrophy[30],entrophy[17]
        ,entrophy[11],data[4],entrophy[2],entrophy[12],entrophy[5],entrophy[7],entrophy[25],data[5],entrophy[4]};

5'd7:first_ivalue_d={entrophy[16],data[0],entrophy[20],entrophy[12],data[5],data[4],entrophy[8],entrophy[26],entrophy[27],entrophy[28],entrophy[21],data[6]
        ,entrophy[30],data[2],data[1],entrophy[0],data[7],entrophy[10],data[3],entrophy[17],entrophy[2],entrophy[19],entrophy[22]
        ,entrophy[3],entrophy[4],entrophy[7],entrophy[11],entrophy[29],entrophy[18],entrophy[1],entrophy[9],entrophy[24]};

5'd8:first_ivalue_d={data[5],entrophy[22],data[4],entrophy[25],entrophy[29],entrophy[28],entrophy[4],data[3],data[7],entrophy[24],entrophy[9]
        ,entrophy[23],data[2],entrophy[12],entrophy[1],entrophy[14],entrophy[5],entrophy[27],entrophy[21],entrophy[15],data[0],entrophy[20],entrophy[7]
        ,data[1],entrophy[2],entrophy[13],entrophy[18],entrophy[10],data[6],entrophy[8],entrophy[19],entrophy[3]};

5'd9:first_ivalue_d={entrophy[24],entrophy[7],entrophy[16],entrophy[4],data[1],entrophy[26],entrophy[15],data[2],entrophy[2],entrophy[6],entrophy[9],entrophy[0],entrophy[13]
        ,entrophy[8],entrophy[27],entrophy[5],entrophy[31],entrophy[3],entrophy[11],entrophy[28],entrophy[18],entrophy[17],entrophy[10],entrophy[25],data[4]
        ,data[6],entrophy[23],data[5],data[7],data[0],entrophy[20],data[3]};

5'd10:first_ivalue_d={entrophy[1],entrophy[20],data[1],entrophy[4],entrophy[21],entrophy[14],entrophy[24],entrophy[0],data[7],entrophy[25],data[2],entrophy[26]
        ,data[5],data[0],data[6],entrophy[2],entrophy[11],entrophy[10],entrophy[8],entrophy[30],entrophy[16],data[4],entrophy[22]
        ,entrophy[31],entrophy[28],entrophy[29],entrophy[3],data[3],entrophy[27],entrophy[23],entrophy[15],entrophy[18]};

5'd11:first_ivalue_d={entrophy[2],entrophy[13],entrophy[23],data[3],data[2],entrophy[18],data[7],entrophy[9],entrophy[26],entrophy[31],entrophy[20],entrophy[0]
        ,entrophy[14],entrophy[6],entrophy[4],data[1],entrophy[1],entrophy[21],data[5],entrophy[16],entrophy[22],data[6],entrophy[28],entrophy[27]
        ,entrophy[7],entrophy[17],entrophy[11],entrophy[8],data[0],data[4],entrophy[15],entrophy[3]};

5'd12:first_ivalue_d={entrophy[25],entrophy[19],entrophy[26],entrophy[21],entrophy[16],entrophy[27],entrophy[28],entrophy[4],data[5],data[3],data[2],entrophy[8]
        ,data[4],entrophy[20],data[7],entrophy[13],data[0],entrophy[31],entrophy[14],entrophy[0],data[6],entrophy[23],entrophy[29]
        ,entrophy[11],data[1],entrophy[9],entrophy[17],entrophy[3],entrophy[22],entrophy[18],entrophy[30],entrophy[7]};

5'd13:first_ivalue_d={entrophy[30],entrophy[19],entrophy[7],entrophy[6],entrophy[21],entrophy[8],entrophy[9],entrophy[4],data[3],data[6],entrophy[27],entrophy[16],entrophy[29]
        ,entrophy[11],entrophy[28],entrophy[15],entrophy[23],data[4],entrophy[5],entrophy[3],entrophy[2],entrophy[31],data[1],data[2],data[7]
        ,entrophy[13],data[5],entrophy[1],entrophy[26],entrophy[10],entrophy[14],data[0]};

5'd14:first_ivalue_d={entrophy[9],entrophy[7],entrophy[23],entrophy[1],entrophy[21],entrophy[28],data[6],entrophy[11],data[1],entrophy[27],entrophy[14],entrophy[22]
        ,entrophy[25],entrophy[19],entrophy[6],entrophy[10],data[3],data[7],entrophy[26],entrophy[4],entrophy[30],entrophy[12],data[4]
        ,entrophy[18],data[0],entrophy[16],entrophy[29],data[5],entrophy[15],entrophy[13],entrophy[3],data[2]};

5'd15:first_ivalue_d={entrophy[17],entrophy[18],entrophy[24],entrophy[27],data[3],data[5],entrophy[2],data[4],data[7],entrophy[14],entrophy[23]
        ,entrophy[1],entrophy[22],entrophy[25],entrophy[16],data[2],entrophy[29],entrophy[8],entrophy[13],data[0],entrophy[30],entrophy[5],entrophy[11]
        ,entrophy[10],data[1],data[6],entrophy[9],entrophy[0],entrophy[21],entrophy[12],entrophy[28],entrophy[20]};

5'd16:first_ivalue_d={entrophy[20],entrophy[6],entrophy[3],entrophy[21],entrophy[9],data[2],entrophy[1],data[4],entrophy[14],entrophy[26],entrophy[19],entrophy[4],entrophy[17]
        ,entrophy[25],entrophy[5],entrophy[31],entrophy[7],entrophy[8],entrophy[15],data[7],data[5],entrophy[27],entrophy[0],entrophy[23],data[1]
        ,entrophy[18],data[6],data[3],data[0],entrophy[10],entrophy[2],entrophy[12]};

5'd17:first_ivalue_d={data[0],data[7],data[3],entrophy[25],entrophy[3],data[2],entrophy[0],entrophy[17],entrophy[11],entrophy[5],entrophy[18],entrophy[13]
        ,entrophy[7],entrophy[9],entrophy[23],entrophy[24],data[1],entrophy[8],entrophy[29],data[5],entrophy[4],entrophy[15],entrophy[31],data[4]
        ,data[6],entrophy[22],entrophy[1],entrophy[6],entrophy[19],entrophy[27],entrophy[12],entrophy[28]};

5'd18:first_ivalue_d={entrophy[20],entrophy[11],entrophy[9],entrophy[2],entrophy[8],entrophy[14],entrophy[3],data[7],entrophy[17],entrophy[12],entrophy[5],entrophy[7],entrophy[16]
        ,data[3],entrophy[25],entrophy[31],entrophy[22],entrophy[24],data[1],data[4],entrophy[19],entrophy[18],data[6],entrophy[15]
        ,entrophy[1],entrophy[21],data[5],data[2],entrophy[27],data[0],entrophy[4],entrophy[10]};

5'd19:first_ivalue_d={entrophy[21],entrophy[24],entrophy[6],entrophy[8],entrophy[26],entrophy[17],entrophy[12],data[2],data[5],entrophy[2],entrophy[18],data[3]
        ,entrophy[15],entrophy[4],entrophy[14],entrophy[0],entrophy[16],entrophy[11],entrophy[22],entrophy[31],entrophy[27],entrophy[5],entrophy[23],entrophy[29],data[4]
        ,entrophy[20],entrophy[25],data[0],entrophy[3],data[7],data[1],data[6]};

5'd20:first_ivalue_d={entrophy[13],data[0],entrophy[25],entrophy[15],entrophy[27],entrophy[28],entrophy[24],entrophy[31],data[1],entrophy[6],data[7],entrophy[10]
        ,data[2],entrophy[2],entrophy[11],entrophy[9],data[3],entrophy[20],entrophy[19],entrophy[5],data[5],entrophy[26],entrophy[7],data[6]
        ,data[4],entrophy[3],entrophy[8],entrophy[29],entrophy[14],entrophy[16],entrophy[23],entrophy[30]};

5'd21:first_ivalue_d={entrophy[5],entrophy[22],entrophy[27],data[6],data[0],entrophy[10],data[3],data[5],entrophy[13],entrophy[20],data[4]
        ,entrophy[23],entrophy[11],entrophy[4],data[2],entrophy[2],entrophy[9],entrophy[19],entrophy[24],entrophy[1],entrophy[29],entrophy[6],entrophy[21],entrophy[30]
        ,entrophy[16],entrophy[18],data[7],entrophy[15],entrophy[17],data[1],entrophy[31],entrophy[14]};

5'd22:first_ivalue_d={entrophy[27],entrophy[17],entrophy[0],entrophy[15],entrophy[22],entrophy[8],entrophy[12],entrophy[29],entrophy[20],entrophy[6],data[2],entrophy[24],entrophy[26]
        ,entrophy[2],entrophy[13],entrophy[4],entrophy[19],data[1],data[5],entrophy[21],entrophy[7],entrophy[9],entrophy[18],data[0],data[3]
        ,data[4],entrophy[5],entrophy[30],entrophy[16],data[6],entrophy[3],data[7]};

5'd23:first_ivalue_d={entrophy[8],entrophy[26],entrophy[30],entrophy[24],entrophy[3],entrophy[19],entrophy[23],entrophy[22],entrophy[16],entrophy[21],entrophy[17],entrophy[10],entrophy[4]
        ,entrophy[18],data[0],entrophy[31],entrophy[1],entrophy[5],entrophy[27],data[5],entrophy[7],entrophy[11],entrophy[2],data[4],entrophy[15]
        ,data[6],data[1],entrophy[6],entrophy[0],data[7],data[2],data[3]};

5'd24:first_ivalue_d={entrophy[17],entrophy[10],entrophy[27],entrophy[22],data[4],entrophy[14],entrophy[4],entrophy[12],entrophy[30],data[5],entrophy[1],data[2]
        ,entrophy[20],entrophy[29],entrophy[19],entrophy[6],entrophy[28],entrophy[18],entrophy[26],entrophy[2],data[7],data[0],data[6]
        ,entrophy[11],entrophy[24],entrophy[5],entrophy[7],data[1],data[3],entrophy[8],entrophy[23],entrophy[25]};

5'd25:first_ivalue_d={entrophy[30],data[2],entrophy[6],entrophy[28],entrophy[8],entrophy[22],data[1],entrophy[10],entrophy[19],entrophy[7],entrophy[25],entrophy[24]
        ,entrophy[31],entrophy[13],data[3],data[7],entrophy[20],entrophy[15],data[4],entrophy[9],data[6],entrophy[2],entrophy[3]
        ,data[5],entrophy[23],entrophy[18],entrophy[11],entrophy[21],entrophy[4],entrophy[0],data[0],entrophy[29]};

5'd26:first_ivalue_d={entrophy[25],entrophy[23],entrophy[9],entrophy[1],data[4],data[5],entrophy[6],entrophy[28],entrophy[7],data[2],entrophy[18],entrophy[0]
        ,entrophy[26],entrophy[10],data[1],data[3],entrophy[22],entrophy[16],entrophy[17],data[7],entrophy[27],entrophy[15],data[0]
        ,entrophy[24],entrophy[5],entrophy[13],entrophy[2],entrophy[19],entrophy[12],data[6],entrophy[4],entrophy[20]};

5'd27:first_ivalue_d={data[0],entrophy[4],entrophy[2],entrophy[19],entrophy[31],entrophy[23],data[7],entrophy[14],entrophy[5],entrophy[16],data[5],entrophy[13]
        ,entrophy[18],entrophy[6],entrophy[3],entrophy[20],entrophy[24],entrophy[25],entrophy[7],entrophy[1],entrophy[21],entrophy[15],data[4],data[2]
        ,entrophy[26],data[3],entrophy[17],entrophy[8],data[1],entrophy[28],data[6],entrophy[10]};

5'd28:first_ivalue_d={entrophy[23],data[5],entrophy[14],entrophy[5],entrophy[21],entrophy[1],entrophy[26],entrophy[4],entrophy[30],entrophy[18],entrophy[25],entrophy[10],entrophy[15]
        ,entrophy[29],data[4],entrophy[13],data[3],entrophy[9],entrophy[24],entrophy[8],data[6],data[1],data[0],entrophy[22]
        ,entrophy[12],entrophy[19],entrophy[0],entrophy[3],data[2],entrophy[31],data[7],entrophy[7]};

5'd29:first_ivalue_d={entrophy[12],entrophy[22],entrophy[14],data[2],entrophy[1],entrophy[29],entrophy[31],entrophy[16],data[6],entrophy[6],entrophy[18],entrophy[7]
        ,entrophy[4],entrophy[9],data[3],entrophy[3],data[1],data[0],entrophy[23],data[7],entrophy[10],entrophy[2],entrophy[0],data[5]
        ,entrophy[15],entrophy[25],entrophy[20],data[4],entrophy[24],entrophy[8],entrophy[19],entrophy[27]};

5'd30:first_ivalue_d={data[6],data[2],entrophy[18],entrophy[22],entrophy[3],entrophy[2],entrophy[10],entrophy[30],entrophy[9],entrophy[12],entrophy[14],entrophy[7]
        ,entrophy[17],entrophy[15],entrophy[26],entrophy[25],entrophy[6],entrophy[20],entrophy[27],entrophy[29],data[0],data[5],entrophy[19],entrophy[13]
        ,data[7],entrophy[16],entrophy[1],entrophy[24],data[3],data[4],entrophy[28],data[1]};

5'd31:first_ivalue_d={entrophy[27],entrophy[22],entrophy[7],entrophy[31],entrophy[9],entrophy[6],entrophy[20],entrophy[21],data[7],data[2],entrophy[15],entrophy[11]
        ,data[1],entrophy[16],entrophy[14],data[6],entrophy[18],entrophy[25],entrophy[30],entrophy[17],entrophy[10],entrophy[0],entrophy[13],entrophy[3]
        ,data[0],data[4],data[3],data[5],entrophy[24],entrophy[23],entrophy[29],entrophy[5]};
        endcase
        end
always @(posedge clk or posedge rst) begin
if (rst) begin
        first_ivalue<=0;
        pushout<=0;
        end
else
        begin
           first_ivalue<=first_ivalue_d;
        if(pushin)
        pushout<=1;
        else
        pushout<=0;
end
end
        endmodule

 module data_selector(clk,rst,addr,write,pushin,wdata,poly4_tap,dataselect_dataout,dselect,pselect);
input clk;
 input rst;
 input [11:0] addr;
 input write;
input pushin;
 input [31:0] wdata;
input  [6:0] poly4_tap;
 output  reg [63:0] dataselect_dataout;
 output   [4:0] dselect;
 output   [4:0] pselect;
wire [4:0] dselect_d,pselect_d;
 reg [63:0] ds;
reg [63:0] tmp;
 wire lfsr_bit;
reg pushout_d;
assign  pselect={dataselect_dataout[55],dataselect_dataout[22],dataselect_dataout[52],dataselect_dataout[56],dataselect_dataout[25]};
assign dselect={dataselect_dataout[32],dataselect_dataout[27],dataselect_dataout[11],dataselect_dataout[59],dataselect_dataout[48]};

 always @(posedge clk or posedge rst) begin
   if(rst)
    begin
	dataselect_dataout<=0;
        end
   else
    begin
	if((write==1'b1) &&(addr==12'h16))
          begin
           dataselect_dataout[31:0]=wdata;
          end     
         if((write==1'b1)&&(addr==12'h17))
          begin
           dataselect_dataout[63:32]=wdata;
          end  
if(pushin) begin
        for (int i=0;i<7;i=i+1) 
         begin
           dataselect_dataout={dataselect_dataout[62],dataselect_dataout[61]^dataselect_dataout[63],dataselect_dataout[60],dataselect_dataout[59],dataselect_dataout[58],dataselect_dataout[57],dataselect_dataout[56]^dataselect_dataout[63],dataselect_dataout[55],dataselect_dataout[54]^dataselect_dataout[63],dataselect_dataout[53]^dataselect_dataout[63],dataselect_dataout[52]^dataselect_dataout[63],dataselect_dataout[51]^dataselect_dataout[63],dataselect_dataout[50],dataselect_dataout[49],dataselect_dataout[48],dataselect_dataout[47],dataselect_dataout[46]^dataselect_dataout[63],dataselect_dataout[45]^dataselect_dataout[63],dataselect_dataout[44]^dataselect_dataout[63],dataselect_dataout[43],dataselect_dataout[42],dataselect_dataout[41],dataselect_dataout[40],dataselect_dataout[39]^dataselect_dataout[63],dataselect_dataout[38]^dataselect_dataout[63],dataselect_dataout[37]^dataselect_dataout[63],dataselect_dataout[36]^dataselect_dataout[63],dataselect_dataout[35],dataselect_dataout[34]^dataselect_dataout[63],dataselect_dataout[33],dataselect_dataout[32]^dataselect_dataout[63],dataselect_dataout[31]^dataselect_dataout[63],dataselect_dataout[30]^dataselect_dataout[63],dataselect_dataout[29],dataselect_dataout[28]^dataselect_dataout[63],dataselect_dataout[27],dataselect_dataout[26]^dataselect_dataout[63],dataselect_dataout[25],dataselect_dataout[24],dataselect_dataout[23]^dataselect_dataout[63],dataselect_dataout[22]^dataselect_dataout[63],dataselect_dataout[21]^dataselect_dataout[63],dataselect_dataout[20]^dataselect_dataout[63],dataselect_dataout[19],dataselect_dataout[18]^dataselect_dataout[63],dataselect_dataout[17],dataselect_dataout[16]^dataselect_dataout[63],dataselect_dataout[15],dataselect_dataout[14],dataselect_dataout[13],dataselect_dataout[12]^dataselect_dataout[63],dataselect_dataout[11]^dataselect_dataout[63],dataselect_dataout[10],dataselect_dataout[09]^dataselect_dataout[63],dataselect_dataout[08]^dataselect_dataout[63],dataselect_dataout[07],dataselect_dataout[06]^dataselect_dataout[63],dataselect_dataout[05],dataselect_dataout[04],dataselect_dataout[03]^dataselect_dataout[63],dataselect_dataout[02],dataselect_dataout[01],dataselect_dataout[00]^dataselect_dataout[63],poly4_tap[i]^dataselect_dataout[63]};
$display ("for i =%d value of dataselect_dataout is %h\n",i,dataselect_dataout);
          end
          end
          end
          end
endmodule 
module data_scrambler(clk,rst,pushin,pdata_out,first_ivalue,scramble_data_out,pushout);
 input clk;
 input rst;
 input pushin;
 input [31:0] first_ivalue;
 input [15:0] pdata_out;
 output  reg  [31:0] scramble_data_out;
 reg pushin;
  output reg pushout;

 reg [31:0] scramble_data_out_d;
assign scramble_data_out_d=scramble_16(first_ivalue,pdata_out);

 always @(posedge clk or posedge rst) 
	begin
   if(rst)
    begin
      scramble_data_out<=32'b0;
      pushout<=0;
	end
   else
   begin
 scramble_data_out<=scramble_data_out_d;
          pushout<=pushin;
	  end
    end

function [31:0] scramble_16(input [31:0] scr,input [15:0]pdata_out);
begin

  scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[0]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
 scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[1]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
 scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[2]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
 scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[3]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[4]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);

scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[5]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[6]^scr[31]};
//$display(" for step 6 value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[7]^scr[31]};
//$display(" for step 7 value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[8]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[9]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[10]^scr[31]};

//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);

scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[11]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[12]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[13]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[14]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);
scr=  {  scr[30]^scr[31],
                  scr[29],scr[28],scr[27],scr[26],scr[25],scr[24],scr[23]^scr[31],scr[22],scr[21]^scr[31],scr[20],
                  scr[19],scr[18],scr[17],scr[16],scr[15]^scr[31],scr[14],scr[13]^scr[31],scr[12],scr[11],scr[10],
           scr[09],scr[08],scr[07]^scr[31],scr[06]^scr[31],scr[05],scr[04]^scr[31],scr[03],scr[02]^scr[31],scr[01],scr[00]^scr[31],pdata_out[15]^scr[31]};
//$display(" value of scr is %h and pdata_out is %b",scr,pdata_out);


	return scr;
          end  
          endfunction 
endmodule 
