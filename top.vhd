library ieee;
use ieee.std_logic_1164.all;    
--use IEEE.std_logic_arith.all;        
use IEEE.numeric_std.all;
--use IEEE.std_logic_unsigned.all;
use work.pcg.all;
        

entity top is

	port 
	(
				CLOCK_50 : in std_logic;
				CLOCK_50_2 : in std_logic;
			
				BUTTON : in std_logic_vector(2 downto 0);		
		--////////////////////	DPDT Switch		////////////////////
			SW : in std_logic_vector(9 downto 0);				
		--////////////////////	7-SEG Dispaly	////////////////////
		HEX0_D : out std_logic_vector(6 downto 0);			
		HEX0_DP : out std_logic;	
		HEX1_D : out std_logic_vector(6 downto 0);			
		HEX1_DP : out std_logic;
		HEX2_D : out std_logic_vector(6 downto 0);								
		HEX2_DP : out std_logic;
		HEX3_D : out std_logic_vector(6 downto 0);									
		HEX3_DP : out std_logic;
		--////////////////////////	LED		////////////////////////
		LEDG : out std_logic_vector(9 downto 0);										
		--////////////////////////	UART	////////////////////////
		UART_TXD	 : out std_logic;
		UART_RXD	 : in std_logic;
		UART_CTS	 : out std_logic;
		UART_RTS		 :in std_logic;
		--/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ				 :inout std_logic_vector(15 downto 0);										
		DRAM_ADDR		 : out std_logic_vector(12 downto 0);										
		DRAM_LDQM			 : out std_logic;
		DRAM_UDQM		 : out std_logic;
		DRAM_WE_N			 : out std_logic;
		DRAM_CAS_N			 : out std_logic;
		DRAM_RAS_N					 : out std_logic;
		DRAM_CS_N				 : out std_logic;
		DRAM_BA_0					 : out std_logic;
		DRAM_BA_1					 : out std_logic;
		DRAM_CLK					 : out std_logic;
		DRAM_CKE					 : out std_logic;
		--////////////////////	Flash Interface		////////////////
		FL_DQ						 : inout std_logic_vector(14 downto 0);										
		FL_DQ15_AM1				 : inout std_logic;
		FL_ADDR					 : out std_logic_vector(21 downto 0);										
		FL_WE_N					 : out std_logic;
		FL_RST_N				 : out std_logic;
		FL_OE_N					 : out std_logic;
		FL_CE_N					 : out std_logic;
		FL_WP_N						 : out std_logic;
		FL_BYTE_N				 : out std_logic;
		FL_RY						 : in std_logic;
--		////////////////////	LCD Module 16X2		////////////////
		LCD_BLON					 :out std_logic;	
		LCD_RW						 : out std_logic;
		LCD_EN						 : out std_logic;
		LCD_RS						 : out std_logic;
		LCD_DATA					 : inout std_logic_vector(7 downto 0);
		--////////////////////	SD_Card Interface	////////////////
		SD_DAT0					 : inout std_logic;
		SD_DAT3						 : inout std_logic;
		SD_CMD						 : inout std_logic;
		SD_CLK						 : out std_logic;
		SD_WP_N					 : in std_logic;
		--////////////////////	PS2		////////////////////////////
		PS2_KBDAT					 : inout std_logic;
		PS2_KBCLK					 : inout std_logic;
		PS2_MSDAT					 : inout std_logic;
		PS2_MSCLK					 : inout std_logic;
		--////////////////////	VGA		////////////////////////////
		VGA_HS						 : out std_logic;
		VGA_VS						 : out std_logic;
		VGA_R   					 : out std_logic_vector(3 downto 0);										
		VGA_G	 					 : out std_logic_vector(3 downto 0);										
		VGA_B  					 : out std_logic_vector(3 downto 0);										
		--////////////////////	GPIO	////////////////////////////
		GPIO0_CLKIN				 : in std_logic_vector(1 downto 0);										
		GPIO0_CLKOUT				 : out std_logic_vector(1 downto 0);										
		GPIO0_D						 : inout std_logic_vector(31 downto 0);										
		GPIO1_CLKIN				 : in std_logic_vector(1 downto 0);										
		GPIO1_CLKOUT					 : out std_logic_vector(1 downto 0);										
		GPIO1_D						 : inout std_logic_vector(31 downto 0)
	);

end entity;


architecture rtl of top  is
	signal is_white: std_logic;
	signal f_address	:  std_logic_vector(8 downto 0);
	signal f_wraddress	:  std_logic_vector(8 downto 0);
	signal f_wren	:  std_logic;
	signal fb0_wren:std_logic;
	signal fb1_wren:std_logic;
	signal s_ps2k_dout: std_logic_vector(7 downto 0);
	signal fb0_clk:std_logic;
	signal fb1_clk:std_logic;
	signal f_wrclock	:  std_logic;
	signal f_q : std_logic_vector(15 downto 0);
	signal f_data : std_logic_vector(15 downto 0);
	signal fb0_data: std_logic_vector(7 downto 0);
	signal fb1_data: std_logic_vector(7 downto 0);
		
	signal DRAM_ADDR2		 :  std_logic_vector(12 downto 0);										
	signal DRAM_ADDR1		 :  std_logic_vector(12 downto 0);										
		
	signal f_ram_dat :  std_logic_vector(15 downto 0);

	signal s_fifo_data:  std_logic_vector(7 downto 0);
	signal s_fifo_empty:  std_logic;
	signal s_fifo_rd:  std_logic;
		
	signal s_outpA:  std_logic_vector(31 downto 0);
	signal s_outpB:  std_logic_vector(31 downto 0);
	signal s_outpC:  std_logic_vector(31 downto 0);
	signal s_outpD:  std_logic_vector(31 downto 0);
		
	signal s_inpA	: std_logic_vector(31 downto 0);
	signal s_inpB	: std_logic_vector(31 downto 0);
	signal s_inpC	: std_logic_vector(31 downto 0);
	constant C_BTN_CLK : integer := 10e6;         	 -- 1MHz
	signal s_clk_btn:std_logic;


	signal DRAM_DQ1				 :std_logic_vector(15 downto 0);										
	signal DRAM_DQ2				 :std_logic_vector(15 downto 0);										
	
	TYPE STAGES IS (ST0,ST1,ST2);
	SIGNAL BUFF_CTRL: STAGES:=ST0;
	---------------------------test signals----------------------------
	signal counter : integer range 0 to 1000;
	--signal test: std_logic:='0';
	signal testdata: std_logic_vector(7 downto 0):="00000000";
	signal Xpos,Ypos: integer range 0 to 799:=0;
	---------------------------sync-----------------------------
	signal BUFF_WAIT: std_logic:='0';
	signal VGAFLAG: std_logic_vector(2 downto 0);
	-------------------------ram/gray----------------------------
	signal RAMFULL_POINTER:integer range 0 to 511:=0;
	signal RAMRESTART_POINTER: integer range 0 to 511:=0;
	signal RAMADDR1GR,RAMADDR2GR: std_logic_vector(8 downto 0):=(others=>'0');
	signal RAMADDR1GR_sync0,RAMADDR1GR_sync1,RAMADDR1GR_sync2,RAMADDR1_bin: std_logic_vector(8 downto 0);
	signal RAMADDR2GR_sync0,RAMADDR2GR_sync1,RAMADDR2GR_sync2,RAMADDR2_bin: std_logic_vector(8 downto 0);
	-------------------------dual ram ----------------------------------
	signal RAMIN1,RAMIN2,RAMOUT1,RAMOUT2: std_logic_vector(15 downto 0);
	signal RAMWE1,RAMWE2: std_logic:='0';
	signal RAMADDR1,RAMADDR2: integer range 0 to 511:=0;
	------------------vga----------------------------------
	signal NEXTFRAME: std_logic_vector(2 downto 0):="000";
	signal FRAMEEND,FRAMESTART: std_logic:='0';
	signal ACTVIDEO: std_logic:='0';
	signal VGABEGIN: std_logic:='0';
	signal RED,GREEN,BLUE: STD_LOGIC_VECTOR(7 downto 0);
	------------------clock--------------------------------
	SIGNAL CLK143_2,CLK49_5: STD_LOGIC;
	signal cpu_clk: std_logic;
	------------------sdram--------------------------------
	SIGNAL SDRAM_ADDR: STD_LOGIC_VECTOR(24 downto 0);
	SIGNAL SDRAM_ADDR1: STD_LOGIC_VECTOR(24 downto 0);
	SIGNAL s_VGA_B: STD_LOGIC_VECTOR(9 downto 0);
	SIGNAL s_VGA_G: STD_LOGIC_VECTOR(9 downto 0);
	SIGNAL s_VGA_R: STD_LOGIC_VECTOR(9 downto 0);
	SIGNAL SDRAM_BE_N: STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL SDRAM_CS: STD_LOGIC;
	SIGNAL SDRAM_RDVAL,SDRAM_WAIT:STD_LOGIC;
	SIGNAL SDRAM_RDVAL1,SDRAM_WAIT1:STD_LOGIC;
	SIGNAL SDRAM_RE_N,SDRAM_WE_N: STD_LOGIC;
	SIGNAL SDRAM_RE_N1,SDRAM_WE_N1: STD_LOGIC;
	SIGNAL SDRAM_READDATA,SDRAM_WRITEDATA: STD_LOGIC_VECTOR(15 downto 0);
	SIGNAL SDRAM_READDATA1,SDRAM_WRITEDATA1: STD_LOGIC_VECTOR(15 downto 0);

	SIGNAL DRAM_DQM : STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL DRAM_DQM1 : STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL DRAM_BA1 : STD_LOGIC_VECTOR(1 downto 0);
	SIGNAL DRAM_BA : STD_LOGIC_VECTOR(1 downto 0);

component  vga is
	port(
		CLK: in std_logic;
		R_OUT,G_OUT,B_OUT: OUT std_logic_vector(7 downto 0);
		R_IN,G_IN,B_IN: IN std_logic_vector(7 downto 0);
		VGAHS, VGAVS:OUT std_logic;
	   ACTVID: OUT std_logic;
		VGA_FRAMESTART: out std_logic;
		VGA_FRAMEEND: out std_logic
	);
end component vga;


component keyb_fifo IS
	PORT
	(
		clock		: IN STD_LOGIC ;
		data		: IN STD_LOGIC_VECTOR (7 DOWNTO 0);
		rdreq		: IN STD_LOGIC ;
		wrreq		: IN STD_LOGIC ;
		empty		: OUT STD_LOGIC ;
		full		: OUT STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
	);
END component;

component ps2_rx is 
port
 ( clk, reset: in std_logic; 
	 ps2d, ps2c: in std_logic;
	 -- key data, key clock 
	 rx_en : in std_logic ;
	 rx_done_tick: out std_logic;
	 dout: out std_logic_vector (7 downto 0) 
 ); 
 end component;
 
component SEG7_LUT_4 IS
	PORT
	 (	
	 	iDIG:in std_logic_vector(15 downto 0);
		oSEG0: out std_LOGIC_VECTOR(6 downto 0);
		oSEG1: out std_LOGIC_VECTOR(6 downto 0);
		oSEG2: out std_LOGIC_VECTOR(6 downto 0);
		oSEG3: out std_LOGIC_VECTOR(6 downto 0);
		oSEG0_DP: out std_logic;
		oSEG1_DP:out std_logic;
		oSEG2_DP:out std_logic;
		oSEG3_DP:out std_logic
	 );
	 END component;

component true_dual_port_ram_dual_clock is
	port 
	(
		clk_a	: in std_logic;
		clk_b	: in std_logic;
		addr_a	: in natural range 0 to 511;
		addr_b	: in natural range 0 to 511;
		data_a	: in std_logic_vector(15 downto 0);
		data_b	: in std_logic_vector(15 downto 0);
		we_a	: in std_logic := '1';
		we_b	: in std_logic := '1';
		q_a		: out std_logic_vector(15 downto 0);
		q_b		: out std_logic_vector(15 downto 0)
	);
end component true_dual_port_ram_dual_clock;
	
component vram IS
	PORT
	(
		data		: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		rdaddress		: IN STD_LOGIC_VECTOR (8 DOWNTO 0);
		rdclock		: IN STD_LOGIC ;
		wraddress		: IN STD_LOGIC_VECTOR (8 DOWNTO 0);
		wrclock		: IN STD_LOGIC  := '1';
		wren		: IN STD_LOGIC  := '0';
		q		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0)
	);
END component;

component font_ram IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (11 DOWNTO 0);
		clock		: IN STD_LOGIC  := '1';
		data		: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		wren		: IN STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0)
	);
END component;

component fb_ram IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (11 DOWNTO 0);
		clock		: IN STD_LOGIC  := '1';
		data		: IN STD_LOGIC_VECTOR (7 DOWNTO 0);
		wren		: IN STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
	);
END component;

	signal s_symb_shift: natural range 0 to 15;
	signal fb_current: natural range 0 to 1;
	signal s_row_shift: natural range 0 to 15;
	signal s_symb_x: natural range 0 to 60;
	signal s_symb_y: natural range 0 to 40;
						
	signal s_font_addr		:  STD_LOGIC_VECTOR (11 DOWNTO 0);		
	signal s_fb0_addr		:  STD_LOGIC_VECTOR (11 DOWNTO 0);		
	signal s_fbX_addr		:  STD_LOGIC_VECTOR (11 DOWNTO 0);		
	signal s_fb1_addr		:  STD_LOGIC_VECTOR (11 DOWNTO 0);		
	signal s_font_q		: STD_LOGIC_VECTOR (15 DOWNTO 0);
	signal s_fb0_q		: STD_LOGIC_VECTOR (7 DOWNTO 0);
	signal s_fbX_q		: STD_LOGIC_VECTOR (7 DOWNTO 0);
	signal s_fb1_q		: STD_LOGIC_VECTOR (7 DOWNTO 0);

component VGA_Ctrl is
port (
	iRed : in std_logic_vector(9 downto 0);
	iGreen : in std_logic_vector(9 downto 0);
	iBlue : in std_logic_vector(9 downto 0);
	oH_c : out std_logic_vector(10 downto 0);
	oCurrent_X : out std_logic_vector(10 downto 0);
	oCurrent_Y : out std_logic_vector(10 downto 0);
	oVGA_R : out std_logic_vector(9 downto 0);
	oVGA_G : out std_logic_vector(9 downto 0);
	oVGA_B : out std_logic_vector(9 downto 0);

	oVGA_HS : out std_logic;
	oVGA_VS : out std_logic;
	oVGA_SYNC : out std_logic;
	oVGA_BLANK : out std_logic;
	oVGA_CLOCK : out std_logic;

	iCLK : in std_logic;
	iRST_N : in std_logic
);
end component;

	signal vga_mode : natural range 0 to 3;
	signal oVGA_HS : std_logic;
	signal oVGA_VS : std_logic;
	signal oVGA_HS1 :  std_logic;
	signal oVGA_VS1 : std_logic;
	signal oVGA_HS2 :  std_logic;
	signal oVGA_VS2 : std_logic;
	
component  gh_clk_ce_div is 
    GENERIC (divide_ratio : natural :=8);
     port(
         CLK : in STD_LOGIC;
         rst : in STD_LOGIC;
         Q : out STD_LOGIC
         );			
end component;


component decoder is	
	port 
	(
		clk		: in std_logic;	
		in_sd_clk		: in std_logic;	
		
		reset		: in  std_logic;
		cmd: in std_logic_vector(3 downto 0);
		cmd_ready : in std_logic;
		cmd_wait : out std_logic;
		
		outflag:out std_logic;
		outp: out std_logic_vector(3 downto 0);
		
		inpA: in std_logic_vector(31 downto 0);
		inpB: in std_logic_vector(31 downto 0);
		inpC: in std_logic_vector(31 downto 0);
		
		outpA: out std_logic_vector(31 downto 0);
		outpB: out std_logic_vector(31 downto 0);
		outpC: out std_logic_vector(31 downto 0);
		outpD: out std_logic_vector(31 downto 0);
		outpE: out std_logic_vector(31 downto 0);
		outpF: out std_logic_vector(31 downto 0);
			--external fifo read interface
		fifo_rd : out std_logic;
		fifo_data: in std_logic_vector(31 downto 0);
		fifo_empty : in std_logic;
		--end fifo
			--sdram
		sdram_s1_address: out STD_LOGIC_VECTOR (21 DOWNTO 0);
		
		
		sdram_s1_writedata: out STD_LOGIC_VECTOR (15 DOWNTO 0);
		sdram_s1_read_n: out STD_LOGIC;
		sdram_s1_write_n: out STD_LOGIC;
		sdram_s1_readdata: in STD_LOGIC_VECTOR (15 DOWNTO 0);
		sdram_s1_readdatavalid: in STD_LOGIC;
		sdram_s1_waitrequest     : in STD_LOGIC;
		--end sdram
		
		--eram
		
		e_ram_addr	: out  natural range 0 to 2**12 - 1;
		e_ram_data	:  out std_logic_vector(31 downto 0);
		--e_ram_q	:  in std_logic_vector(31 downto 0);
		e_ram_we		:  out std_logic := '1';
		
		--end eram
	-- external SD CARD interface
	SD_DAT0 		: inout std_logic;		
	SD_DAT3		: inout std_logic;		
	SD_CMD 		: inout std_logic;		
	SD_CLK 		: out std_logic;		
	SD_WP_N		: in std_logic			

	);

end component;

component VGA_CLK is
port (
	--sd card spi signals
	inclk0 : in std_logic;
	c0 : out std_logic;
	--c1 : out std_logic;
	c2 : out std_logic;
	c3 : out std_logic--143
	--c4 : out std_logic--143  -52
	
);
end component;
  
	signal but:std_logic_vector(2 downto 0);
	signal 	csr_a: std_logic_vector(2 downto 0);
	signal	csr_we: std_logic;
	signal	csr_dw:  std_logic_vector(15 downto 0);
	signal csr_do:  std_logic_vector(15 downto 0);
		
	
	signal	fml_adr: std_logic_vector(22 downto 0);
	signal	fml_stb: std_logic;
	signal	fml_we: std_logic;
	signal	fml_ack: std_logic;
	signal	fml_sel: std_logic_vector(1 downto 0);
	signal	fml_di: std_logic_vector(15 downto 0);
	signal	fml_do: std_logic_vector(15 downto 0);
	
	signal s_flash_data		:  std_logic_vector(15 downto 0);
	signal s_az_addr		:  std_logic_vector(21 downto 0);
   signal s_az_be_n		:  std_logic_vector(1 downto 0);    
   signal s_az_cs		:  std_logic;      
   signal s_az_data		:  std_logic_vector(15 downto 0);    
   signal s_az_rd_n		:  std_logic;
   signal s_az_wr_n		:  std_logic; 	
   signal s_clk		:  std_logic;                           
   signal s_reset_n		:  std_logic; 	                                                                 
                                   
	signal s_za_data		:  std_logic_vector(15 downto 0); 
	signal s_za_valid		:  std_logic;
	signal s_za_waitrequest		:  std_logic;                             
	signal s_zs_addr		:  std_logic_vector(11 downto 0); 
	signal s_zs_ba		:  std_logic_vector(1 downto 0);  
	signal s_zs_cas_n		:  std_logic;  
	signal s_zs_cke		:  std_logic;  
	signal s_zs_cs_n		:  std_logic;
	signal s_zs_dq		:  std_logic_vector(15 downto 0); 
   signal s_zs_dqm		:  std_logic_vector(1 downto 0);                   
   signal s_zs_ras_n		:  std_logic_vector(1 downto 0);                                 
   signal s_zs_we_n		:  std_logic  ;	  
	signal sdram_master:std_logic;
	  
	
component ram1p_2 IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (12 DOWNTO 0);
		clock		: IN STD_LOGIC  := '1';
		data		: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		wren		: IN STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0)
	);
END component;

component ram2p IS
	PORT
	(
		address_a		: IN STD_LOGIC_VECTOR (7 DOWNTO 0);
		address_b		: IN STD_LOGIC_VECTOR (7 DOWNTO 0);
		clock		: IN STD_LOGIC  := '1';
		data_a		: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		data_b		: IN STD_LOGIC_VECTOR (15 DOWNTO 0);
		wren_a		: IN STD_LOGIC  := '0';
		wren_b		: IN STD_LOGIC  := '0';
		q_a		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0);
		q_b		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0)
	);
END component;
	
component sd_controller is
port (

	--sd card spi signals
	cs : out std_logic;
	mosi : out std_logic;
	miso : in std_logic;
	sclk : out std_logic;
	-- end sd card spi signals
	
	--output ram write signals		
		ram_addr	: out natural range 0 to 2**9 - 1;
		ram_data	: out std_logic_vector((8-1) downto 0);
		ram_we		: out std_logic ;
		
	--end output ram write signals
	
	addr : in std_logic_vector(31 downto 0);
	rd : in std_logic;
	
	
	reset : in std_logic;
	
	dout : out std_logic_vector(7 downto 0);	
	read_flag : out std_logic;	
	data_ready: out std_logic;	
	clk : in std_logic	-- twice the SPI clk
);
end component;

component adder is
port (
        input_a:in std_logic_vector(31 downto 0);
        input_b:in std_logic_vector(31 downto 0);
        input_a_stb:in std_logic;
        input_b_stb:in std_logic;
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic;
        input_b_ack:out std_logic
		  
  );
end component;


component multiplier is
port (
        input_a:in std_logic_vector(31 downto 0);
        input_b:in std_logic_vector(31 downto 0);
        input_a_stb:in std_logic;
        input_b_stb:in std_logic;
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic;
        input_b_ack:out std_logic
		  
  );
end component;

component divider is
port (
        input_a:in std_logic_vector(31 downto 0);
        input_b:in std_logic_vector(31 downto 0);
        input_a_stb:in std_logic;
        input_b_stb:in std_logic;
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic;
        input_b_ack:out std_logic
		  
  );
end component;

component int_to_float is
port (
        input_a:in std_logic_vector(31 downto 0);
        
        input_a_stb:in std_logic;
        
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic        		  
  );  
end component;

component debouncer is	
	port 
	(
		clk		: in std_logic;			
		inp		: in  std_logic;
		outp: out std_logic
		
	);
end component;

component ramsys is
        port (
            clk_clk             : in    std_logic                     := 'X';             -- clk
            reset_reset_n       : in    std_logic                     := 'X';             -- reset_n
            clk143_shift_clk    : out   std_logic;                                        -- clk
            clk143_clk          : out   std_logic;                                        -- clk
            clk49_5_clk         : out   std_logic;                                        -- clk
            wire_addr           : out   std_logic_vector(12 downto 0);                    -- addr
            wire_ba             : out   std_logic_vector(1 downto 0);                     -- ba
            wire_cas_n          : out   std_logic;                                        -- cas_n
            wire_cke            : out   std_logic;                                        -- cke
            wire_cs_n           : out   std_logic;                                        -- cs_n
            wire_dq             : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
            wire_dqm            : out   std_logic_vector(1 downto 0);                     -- dqm
            wire_ras_n          : out   std_logic;                                        -- ras_n
            wire_we_n           : out   std_logic;                                        -- we_n
            sdram_address       : in    std_logic_vector(24 downto 0) := (others => 'X'); -- address
            sdram_byteenable_n  : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- byteenable_n
            sdram_chipselect    : in    std_logic                     := 'X';             -- chipselect
            sdram_writedata     : in    std_logic_vector(15 downto 0) := (others => 'X'); -- writedata
            sdram_read_n        : in    std_logic                     := 'X';             -- read_n
            sdram_write_n       : in    std_logic                     := 'X';             -- write_n
            sdram_readdata      : out   std_logic_vector(15 downto 0);                    -- readdata
            sdram_readdatavalid : out   std_logic;                                        -- readdatavalid
            sdram_waitrequest   : out   std_logic                                         -- waitrequest
        );
end component ramsys;
	 
	signal	 wb_clk : std_logic;
	signal	 clk143 : std_logic;
	signal	 clk143_shifted : std_logic;
	signal cntr:integer;
	
	signal s_ps2keyb_reset:std_logic;
	signal s_rx_done_tick:std_logic;
	signal s_rx_en:std_logic;
	
	signal clk : std_logic;
	signal s_rd : std_logic;
	signal s_read_flag : std_logic;
	signal s_data_ready : std_logic;
	signal	 s_wr : std_logic;
	signal	 s_dm_in : std_logic;	-- data mode, 0 = write continuously, 1 = write single block
	
	signal	 s_din :  std_logic_vector(7 downto 0);
	signal	 s_dout :  std_logic_vector(7 downto 0);
	signal	 ramd :  std_logic_vector(7 downto 0);
	signal	 s_dout_clk :  std_logic;
	signal	 s_address :  std_logic_vector(31 downto 0);
	signal s_ram1_dat :  std_logic_vector(15 downto 0);
	
	signal		s_ram_addr	:  natural range 0 to 2**9 - 1;
	signal s_ram_data	: std_logic_vector(7 downto 0);
	signal s_ram_we		:  std_logic := '1';
	signal s_ram_q		:  std_logic_vector(7 downto 0);
	signal sd_reset		:  std_logic;
	signal s_ram_addr2		:  std_logic_vector(7 downto 0);
	signal s_ram_addr3		:  std_logic_vector(7 downto 0);
	signal s_ram_addr_4		:  std_logic_vector(7 downto 0);
	signal s_ram_addr_5		:  std_logic_vector(12 downto 0);

	signal s_color: std_logic_vector(9 downto 0);
	signal s_color1: std_logic_vector(9 downto 0);
	signal s_color2: std_logic_vector(9 downto 0);

	signal s_dout_taken :  std_logic;
	signal s_dout_avail :  std_logic;
	signal cntr22 :  natural;
	signal VGA_CTRL_CLK: std_logic;
	signal s_output_z_ack:std_logic;
	signal sd_clk0: std_logic;

	signal s_vga_data1 : std_logic_vector(15 downto 0);
	signal s_vga_data2 : std_logic_vector(15 downto 0);
	signal s_vga_q : std_logic_vector(15 downto 0);
	
	
	signal mH_c:  std_logic_vector(10 downto 0);

	signal mVGA_X:  std_logic_vector(10 downto 0);
	signal		mVGA_Y:  std_logic_vector(10 downto 0);
	signal		mVGA_R:  std_logic_vector(9 downto 0);
	signal		mVGA_G:  std_logic_vector(9 downto 0);
	signal		mVGA_B:  std_logic_vector(9 downto 0);
		
	signal		sVGA_R:  std_logic_vector(9 downto 0);
	signal		sVGA_G:  std_logic_vector(9 downto 0);
	signal		sVGA_B:  std_logic_vector(9 downto 0);
	
	signal s_input_a_stb: std_logic;
			
	signal RAM_CLK: std_logic;
	signal s_wren:std_logic;
	signal shift:natural;
	signal block_index:natural;
	signal block_index_y:natural;
	signal block_index_x:natural;
	signal s_fl_oe_n:std_logic;
	signal	s_output_z:std_logic_vector(31 downto 0);
	signal	s_output_z2:std_logic_vector(31 downto 0);
	signal	s_output_z1:std_logic_vector(31 downto 0);
	signal		s_wb_dat_0:  std_logic_vector(15 downto 0);
	signal s_dqm:std_logic_vector(1 downto 0);
	signal s_sdram_ba:std_logic_vector(1 downto 0);
					
					
	signal s_address_to_the_cfi_flash: std_LOGIC_VECTOR(21 downto 0);
	signal  s_zs_dqm_from_the_sdram :  STD_LOGIC_VECTOR (1 DOWNTO 0);

	signal s_zs_ba_from_the_sdram :  STD_LOGIC_VECTOR (1 DOWNTO 0);
	
	signal system_reset_n:std_logic:='1';
	signal s_out_port_from_the_seg7: STD_LOGIC_VECTOR (31 DOWNTO 0);
	signal s_UART_CTS : std_logic;
	signal s_UART_TXD: std_logic;
	signal but_reset:std_logic:='0';
					
					
	signal	s_input_a1:std_logic_vector(31 downto 0);			
	signal	s_input_b1:std_logic_vector(31 downto 0);		
	signal DRAM_RAS_N2,DRAM_RAS_N1:std_logic;
	signal DRAM_CAS_N2,DRAM_CAS_N1:std_logic;
	signal DRAM_WE_N2,DRAM_WE_N1:std_logic;
	signal DRAM_CS_N2,DRAM_CS_N1:std_logic;
	signal DRAM_CKE2,DRAM_CKE1:std_logic;
	
	signal s_sdram_s1_address:  STD_LOGIC_VECTOR (21 DOWNTO 0);
	signal s_sdram_s1_byteenable_n:  STD_LOGIC_VECTOR (1 DOWNTO 0);
	signal s_sdram_s1_chipselect:  STD_LOGIC;
	signal s_sdram_s1_writedata:  STD_LOGIC_VECTOR (15 DOWNTO 0);
	signal s_sdram_s1_read_n:  STD_LOGIC;
	signal s_sdram_s1_write_n:  STD_LOGIC;
	signal s_sdram_s1_readdata:  STD_LOGIC_VECTOR (15 DOWNTO 0);
	signal s_sdram_s1_readdatavalid:  STD_LOGIC;
	signal s_sdram_s1_waitrequest     :  STD_LOGIC;
	signal s_pll_c1_clk:  STD_LOGIC;
	
		
					-----------------ram-
	signal s_e_ram_addr	:   natural range 0 to 2**12 - 1;
	signal s_e_ram_data	:   std_logic_vector(31 downto 0);
					------------end ram
					
begin

u1:VGA_CLK	
	port  map		
		(	inclk0=>CLOCK_50,
			c0=>VGA_CTRL_CLK,--40MHz
				--c1=>RAM_CLK,--200MHz
				c2=>wb_clk,--20Mhz
				c3=>cpu_clk--143
				--c4=>clk143_shifted--143 shifted
		);
		
--clk143<=s_pll_c1_clk;
s_sdram_s1_byteenable_n<="00";
s_sdram_s1_chipselect<='1';

s0:SEG7_LUT_4
port map
(
	iDIG=>s_outpB(15 downto 0),
		oSEG0=>HEX0_D,
		oSEG1=>HEX1_D,
		oSEG2=>HEX2_D,
		oSEG3=>HEX3_D,
		oSEG0_DP=>HEX0_DP,
		oSEG1_DP=>HEX1_DP,
		oSEG2_DP=>HEX2_DP,
		oSEG3_DP=>HEX3_DP
);
--


uu0 : component ramsys
        port map (
            clk_clk             => CLOCK_50,             --          clk.clk
            reset_reset_n       => '1',       --        reset.reset_n
            clk143_shift_clk    => CLK143_2,    -- clk143_shift.clk
            clk143_clk          => CLK143,          --       clk143.clk
            clk49_5_clk         => CLK49_5,         --      clk49_5.clk
            wire_addr           => DRAM_ADDR,           --         wire.addr
            wire_ba             => DRAM_BA,             --             .ba
            wire_cas_n          => DRAM_CAS_N,          --             .cas_n
            wire_cke            => DRAM_CKE,            --             .cke
            wire_cs_n           => DRAM_CS_N,           --             .cs_n
            wire_dq             => DRAM_DQ,             --             .dq
            wire_dqm            => DRAM_DQM,            --             .dqm
            wire_ras_n          => DRAM_RAS_N,          --             .ras_n
            wire_we_n           => DRAM_WE_N,           --             .we_n
            sdram_address       => SDRAM_ADDR,       --        sdram.address
            sdram_byteenable_n  => SDRAM_BE_N,  --             .byteenable_n
            sdram_chipselect    => SDRAM_CS,    --             .chipselect
            sdram_writedata     => SDRAM_WRITEDATA,     --             .writedata
            sdram_read_n        => SDRAM_RE_N,        --             .read_n
            sdram_write_n       => SDRAM_WE_N,       --             .write_n
            sdram_readdata      => SDRAM_READDATA,      --             .readdata
            sdram_readdatavalid => SDRAM_RDVAL, --             .readdatavalid
            sdram_waitrequest   => SDRAM_WAIT    --             .waitrequest
        );
		  

		sdram_master<= s_outpC(3);
		SDRAM_WRITEDATA	<=SDRAM_WRITEDATA1 	when sdram_master='1' 	else s_sdram_s1_writedata;
		SDRAM_WE_N<=  		  SDRAM_WE_N1   		when sdram_master='1'  	else s_sdram_s1_write_n;
		SDRAM_RE_N<=  	 	  SDRAM_RE_N1  		when sdram_master='1'  	else s_sdram_s1_read_n;		
		SDRAM_ADDR<=	  	  SDRAM_ADDR1 			when sdram_master='1'	else "000"&s_sdram_s1_address;

		

DRAM_BA_0<=DRAM_BA(0);
DRAM_BA_1<=DRAM_BA(1);

DRAM_CLK<=CLK143_2 ;


SDRAM_CS<='1';
SDRAM_BE_N<="00";
DRAM_ADDR2(12)<='0';


 s_az_be_n<="00";
 s_az_cs<='1';



		--LEDG(3)<=s_za_waitrequest;
		--LEDG(4)<=s_za_valid;
--  
s_inpA(9 downto 0)<=SW;
s_inpB(2 downto 0)<=but;
s_inpC(0)<='0' when fb_current=0 else '1';
s_inpC(1)<=s_rx_done_tick;

LEDG(3 downto 0)<=s_outpA(3 downto 0);
LEDG(8)<='1' when to_integer(unsigned(s_outpA))>0 else '0';
--vga_mode<=to_integer(unsigned(s_outpC(1 downto 0)));
--vga_mode<=to_integer(unsigned(SW(9 downto 8)));
vga_mode<=to_integer(unsigned(s_outpD(1 downto 0)));

s_rx_en<=not s_outpC(2);

bd0: decoder 	port map
	(
		
		clk=>cpu_clk,
		in_sd_clk=>wb_clk,
		
		reset		=> but(0),
		--cmd=>SW(3 downto 0),
		cmd=>"0100",
		cmd_ready =>but(1),
		
		outflag=>GPIO0_D(0),
		--outp=>LEDG(3 downto 0),
		outpA=>s_outpA,
		outpB=>s_outpB,
		outpC=>s_outpC,
		outpD=>s_outpD,
		
---		outpD=>s_outpD,
	--	outpE=>s_outpE,
		--outpF=>s_outpF,
		
			--external fifo read interface
		fifo_rd =>s_fifo_rd,
		fifo_data=>x"000000"&s_fifo_data,
		fifo_empty =>s_fifo_empty,
		--end fifo
		
		
		---------sdram
			
		sdram_s1_address=>s_sdram_s1_address,
		
		
		sdram_s1_writedata=>s_sdram_s1_writedata,
		sdram_s1_read_n=>s_sdram_s1_read_n,
		sdram_s1_write_n=>s_sdram_s1_write_n,
		
--		sdram_s1_readdata=>s_sdram_s1_readdata,
--		sdram_s1_readdatavalid=>s_sdram_s1_readdatavalid,
--		sdram_s1_waitrequest     =>s_sdram_s1_waitrequest,
		
		sdram_s1_readdata=>SDRAM_READDATA,
		sdram_s1_readdatavalid=>SDRAM_RDVAL,
		sdram_s1_waitrequest     =>SDRAM_WAIT,
		
		
		---end sdram
		e_ram_addr=>s_e_ram_addr,--f_wraddress,
		e_ram_data=>s_e_ram_data,
		--e_ram_q=>,
		e_ram_we=>f_wren,
		
		
		inpA=>s_inpA,
		inpB=>s_inpB,
		inpC=>s_inpC,
		
		
			-- external SD CARD interface
	SD_DAT0 	=>SD_DAT0,
	SD_DAT3		=>SD_DAT3,
	SD_CMD 		=>SD_CMD,
	SD_CLK 		=>SD_CLK,
	SD_WP_N		=>SD_WP_N	
		
	);
	
	
	
	
	f_ram_dat<=s_e_ram_data(15 downto 0);
	f_wraddress<=std_logic_vector(to_unsigned(s_e_ram_addr, f_wraddress'length));
	DIV_BTN: gh_clk_ce_div
    GENERIC map (divide_ratio => C_BTN_CLK)
     port map(
         CLK => CLOCK_50,
         rst => '1',
         Q => s_clk_btn 
    );
	 gen: for I in 0 to 2 generate
      bl : debouncer port map
        (   clk => s_clk_btn,
         inp => BUTTON(I),
         outp => but(I)
			
			);
   end generate gen;
	f_wrclock<=clk143;
	
	-----------------vga controller = framebuffer

	
--vga_mode<= 1 when SW(9)='0'  else 2;
VGA_R<=sVGA_R(3 downto 0) when vga_mode=1 else s_vGA_R(3 downto 0) when vga_mode=2 else "0000";
VGA_G<=sVGA_G(3 downto 0) when vga_mode=1 else s_vGA_G(3 downto 0) when vga_mode=2  else "0000";
VGA_B<=sVGA_B(3 downto 0) when vga_mode=1 else s_vGA_B(3 downto 0) when vga_mode=2 else "0000";
--VGA_B<=s_VGA_B(3 downto 0);
--VGA_G<=s_VGA_G(3 downto 0);
--VGA_R<=s_VGA_R(3 downto 0);

VGA_HS<=oVGA_HS1 when vga_mode=1 else oVGA_HS2   when vga_mode=2 else '0';
VGA_VS<=oVGA_VS1 when vga_mode=1 else oVGA_VS2 when vga_mode=2 else '0';
		
u2:VGA_Ctrl	
port  map	
		(	
			oCurrent_X=>mVGA_X,
			oCurrent_Y=>mVGA_Y,
			--iRed=>mVGA_R,
			--iGreen=>mVGA_G,
			--iBlue=>mVGA_B,
			--iRed=>"000000"&s_vga_data(3 downto 0),
			--iGreen=>"000000"&s_vga_data(7 downto 4),
			--iBlue=>"000000"&s_vga_data(11 downto 8),
			iRed=>s_color,
			iGreen=>s_color,
			iBlue=>s_color,
			oH_c=>mH_c,
			
			oVGA_R=>sVGA_R,
			oVGA_G=>sVGA_G,
			oVGA_B=>sVGA_B,
			oVGA_HS=>oVGA_HS1,
			oVGA_VS=>oVGA_VS1,
			
			
			iCLK=>VGA_CTRL_CLK,
			iRST_N=>BUTTON(0)
		);

			

	-----------
	
	font0: font_ram 
	PORT map
	(
		address=>s_font_addr,
		clock		=>VGA_CTRL_CLK,
		data		=>"0000000000000000",
		wren		=>'0',
		q		=>s_font_q
	);
		
	fb0: fb_ram 
	PORT map
	(
		address=>s_fb0_addr,
		clock		=>fb0_clk,
		data		=>fb0_data,
		wren		=>fb0_wren,
		q		=>s_fb0_q
	);
	
	fb1: fb_ram 
	PORT map
	(
		address=>s_fb1_addr,
		clock		=>fb1_clk,
		data		=>fb1_data,
		wren		=>fb1_wren,
		q		=>s_fb1_q
	);
	
keyb0: ps2_rx  
port map
 ( clk=>cpu_clk, 
	reset=>not s_ps2keyb_reset,
	 ps2d=> PS2_KBDAT,
	 ps2c=>PS2_KBCLK,
	 -- key data, key clock 
	 rx_en =>s_rx_en,
	 rx_done_tick=>s_rx_done_tick,
	 dout=>s_ps2k_dout
 ); 
 
 
	s_fbX_q<=s_fb1_q when fb_current=1 else s_fb0_q;
	s_color<="1111111111" when s_font_q(s_symb_shift)='1' else "0000000000";
	s_font_addr<=std_logic_vector(to_unsigned(to_integer(unsigned(s_fbX_q)*16+s_row_shift),s_font_addr'length));
	s_row_shift<= to_integer(unsigned(mVGA_Y(3 downto 0))) ;
	s_symb_shift<= to_integer(unsigned(mVGA_X(3 downto 0))+4) ;
	s_symb_x<=to_integer(unsigned(mVGA_X(10 downto 4)));
	s_symb_y<=to_integer(unsigned(mVGA_Y(10 downto 4)));
	s_fbX_addr<=std_logic_vector(to_unsigned(s_symb_x+s_symb_y*50,s_fbX_addr'length));
	
	
	--fb_current<=1 when SW(3)='1' else 0;
	fb_current<=1 when s_outpC(1)='1' else 0;
	
	fb0_clk<=VGA_CTRL_CLK when fb_current=0 else cpu_clk;
	fb1_clk<=VGA_CTRL_CLK when fb_current=1 else cpu_clk;
	fb0_wren<='0' when fb_current=0 else f_wren;
	fb1_wren<='0' when fb_current=1 else f_wren;
	fb0_data<=s_e_ram_data(7 downto 0);
	fb1_data<=s_e_ram_data(7 downto 0);
	s_fb0_addr<=s_fbX_addr when fb_current=0 else std_logic_vector(to_unsigned(s_e_ram_addr, s_fb0_addr'length));
	s_fb1_addr<=s_fbX_addr when fb_current=1 else std_logic_vector(to_unsigned(s_e_ram_addr, s_fb1_addr'length));
	
	LEDG(5)<='1' when to_integer(unsigned(s_ps2k_dout))>0 else '0';
	
	process(cpu_clk)
	begin
	if rising_edge(clk143) then	
		cntr<=cntr+1;
		if(cntr>1000)then
			s_ps2keyb_reset<='1';		
		end if;
	end if;
	end process;
	
	
kfifo0: keyb_fifo 
	PORT map
	(
		clock		=>cpu_clk,
		data		=>s_ps2k_dout,
		rdreq		=>s_fifo_rd,
		wrreq		=>s_rx_done_tick,
		empty		=>s_fifo_empty,
		--full		: OUT STD_LOGIC ;
		q		=>s_fifo_data
	);
	
	-----------------------------sdram framebuffer
	
u3: component true_dual_port_ram_dual_clock
		   port map  (
			clk_a=>CLK143,
			clk_b=>CLK49_5,
			addr_a=>RAMADDR1,
			addr_b=>RAMADDR2,
			data_a=>RAMIN1,
			data_b=>RAMIN2,
			we_a=>RAMWE1,
			we_b=>RAMWE2,
			q_a=>RAMOUT1,
			q_b=>RAMOUT2			
			);
			
PROCESS (CLK143)
variable test_r: std_logic:='0';
variable test_g: std_logic:='0';
variable test_b: std_logic:='0';
begin
if rising_edge(clk143)then
------------double flop sync----------------------
	RAMADDR2GR_sync0<=RAMADDR2GR;
	RAMADDR2GR_sync1<=RAMADDR2GR_sync0;
	RAMADDR2_bin<=gray_to_bin(RAMADDR2GR_sync1);
   NEXTFRAME(1)<=NEXTFRAME(0);
	NEXTFRAME(2)<=NEXTFRAME(1);


RAMADDR1GR<=bin_to_gray(std_logic_vector(to_unsigned(RAMADDR1,9)));
----------------------------------------------------
	case BUFF_CTRL is
		when st0=>------------write image to  SDRAM     
		if (SDRAM_WAIT='0')then	
		    SDRAM_WE_N1<='0';
			 SDRAM_RE_N1<='1';
------------------------circle generation------------------
			if(Xpos<799)then
				Xpos<=Xpos+1;
				else
				Xpos<=0;
				  if(Ypos<599)then
				  Ypos<=Ypos+1;
				  else
				  Ypos<=0;
				  end if;	  
			end if;
			if(Xpos>600) then
			test_g:='1';
			test_r:='0';
			test_b:='0';
			else
			test_g:='0';
				IF((Xpos-to_integer(unsigned(SW)))*(Xpos-to_integer(unsigned(SW)))+(Ypos-300)*(Ypos-300)<40000)THEN
					test_b:='0';
					test_r:='1';
				else
					test_b:='1';
					test_r:='0';
				end if;
			end if;
------------------------------------------------------------
	       --SDRAM_WRITEDATA(3 downto 0)<=(others=>test_r); 
			 SDRAM_WRITEDATA1(3 downto 0)<=std_logic_vector(to_unsigned(XPos,16))(9 downto 6); 
			 SDRAM_WRITEDATA1(7 downto 4)<=(others=>'0'); 
			 SDRAM_WRITEDATA1(11 downto 8)<=(others=>'0'); 
	       SDRAM_ADDR1<=std_logic_vector(unsigned(SDRAM_ADDR1)+1);	

		end if;	
		
      if(to_integer(unsigned(SDRAM_ADDR1))>(800*600-1) )then-----800x600 resolution
		   RAMADDR1<=0;
			BUFF_WAIT<='0';
			RAMFULL_POINTER<=10;----------min. value 2
		   BUFF_CTRL<=st1;
			SDRAM_ADDR1<=(others=>'0');
		end if;
		when st2=>
			  RAMADDR1<=0;
				BUFF_WAIT<='0';
				RAMFULL_POINTER<=10;----------min. value 2
				BUFF_CTRL<=st1;
				SDRAM_ADDR1<=(others=>'0');
		when st1=>-----------write from SDRAM to BUFFER
		      SDRAM_WE_N1<='1';
            RAMWE1<=SDRAM_RDVAL;
			IF(BUFF_WAIT='0')then
					 SDRAM_RE_N1<='0';
					   ------------if no wait request is issued and read enable------
		            IF(SDRAM_WAIT='0' and SDRAM_RE_N1='0')THEN	
							IF(RAMFULL_POINTER<511)then-----move full pointer
								RAMFULL_POINTER<=RAMFULL_POINTER+1;
								else
								RAMFULL_POINTER<=0;
							end if;			
			            SDRAM_ADDR1<=std_logic_vector(unsigned(SDRAM_ADDR1)+1);		
	               END IF;
						-------------check if the buffer is full----------------------
						IF(to_integer(unsigned(RAMADDR2_bin))=(RAMFULL_POINTER))then
								VGAFLAG(0)<='1';---------init displaying image
								SDRAM_RE_N1<='1';
								BUFF_WAIT<='1';
								IF((RAMADDR2+63)<511)THEN
									RAMRESTART_POINTER<=to_integer(unsigned(RAMADDR2_bin))+63;
									ELSE
									RAMRESTART_POINTER<=to_integer(unsigned(RAMADDR2_bin))+63-511;
								END IF;
						end if;
			END IF;
			    	RAMIN1<=SDRAM_READDATA(15 downto 0);	
					------------while data is avalable, write to buffer RAM
					IF(SDRAM_RDVAL='1')then
						IF(RAMADDR1<511)then
						RAMADDR1<=RAMADDR1+1;
						else
						RAMADDR1<=0;
						end if;
					END IF;
					-------------------------------refill buffer------------------------
					     IF(to_integer(unsigned(RAMADDR2_bin))=RAMRESTART_POINTER and BUFF_WAIT='1')then
						  BUFF_WAIT<='0';		  
						  end if;
					-------------------------------end of frame--------------------------
 				        IF(NEXTFRAME(2)='1')THEN
						      xpos<=0;
	                    	ypos<=0;
								
							   BUFF_CTRL<=ST2;
								VGAFLAG(0)<='0';
								SDRAM_ADDR1<=(others=>'0');
								------------
								counter<=0;
								test_r:='0';
								test_g:='0';
								test_b:='0';
							END IF;
		    
		when others=>NULL;
		END CASE;
end if;
end process;

PROCESS(CLK49_5)
begin
if rising_edge(CLK49_5)then
	 
RAMADDR2GR<=bin_to_gray(std_logic_vector(to_unsigned(RAMADDR2,9)));
-------------dual clock sync-------------------------
RAMADDR1GR_sync0<=RAMADDR1GR;
RAMADDR1GR_sync1<=RAMADDR1GR_sync0;
VGAFLAG(1)<=VGAFLAG(0);
VGAFLAG(2)<=VGAFLAG(1);

RAMADDR1_bin<=gray_to_bin(RAMADDR1GR_sync1);


    IF(VGAFLAG(2)='1' AND FRAMESTART='1' )THEN-------if buffer is rdy and  begin of new frame, start displaying image
	 VGABEGIN<='1';
	 end if;
	 
	 IF(FRAMEEND='1' AND VGABEGIN='1')THEN------end of frame
	 NEXTFRAME(0)<='1';
	 VGABEGIN<='0';
	 ELSE
	 NEXTFRAME(0)<='0';
	 END IF;
	
		IF(ACTVIDEO='1'AND to_integer(unsigned(RAMADDR1_bin))/=RAMADDR2  AND VGABEGIN='1')then----if buffer ia not empty
			IF(RAMADDR2<511)then
			RAMADDR2<=RAMADDR2+1;
			else
			RAMADDR2<=0;
			end if;
		   RED<="0000"&RAMOUT2(3 downto 0);
	      GREEN<="0000"&RAMOUT2(7 downto 4);
		   BLUE<="0000"&RAMOUT2(11 downto 8);
		ELSIF(VGABEGIN='0')THEN---------if buffer not ready
	   RAMADDR2<=0;
		BLUE<=(others=>'0');
		RED<=(others=>'0');
		GREEN<=(others=>'0');
		END IF;
end if;
end process;

uu1 : component vga 
			port map(
					CLK=>CLK49_5,
					R_OUT=>s_VGA_R(7 downto 0),
					G_OUT=>s_VGA_G(7 downto 0),
					B_OUT=>s_VGA_B(7 downto 0),
					R_IN=>RED,
					G_IN=>GREEN,
					B_IN=>BLUE,
					VGAHS=>oVGA_HS2,
					VGAVS=>oVGA_VS2,
				   ACTVID=>ACTVIDEO,
					VGA_FRAMESTART=>FRAMESTART,
					VGA_FRAMEEND=>FRAMEEND
			);
			
end rtl;

