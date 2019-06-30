library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.math_real.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

entity rt_reconf is
	port (
		-- OSIF FIFO ports
		OSIF_Sw2Hw_Data    : in  std_logic_vector(31 downto 0);
		OSIF_Sw2Hw_Empty   : in  std_logic;
		OSIF_Sw2Hw_RE      : out std_logic;

		OSIF_Hw2Sw_Data    : out std_logic_vector(31 downto 0);
		OSIF_Hw2Sw_Full    : in  std_logic;
		OSIF_Hw2Sw_WE      : out std_logic;

		-- MEMIF FIFO ports
		MEMIF_Hwt2Mem_Data    : out std_logic_vector(31 downto 0);
		MEMIF_Hwt2Mem_Full    : in  std_logic;
		MEMIF_Hwt2Mem_WE      : out std_logic;

		MEMIF_Mem2Hwt_Data    : in  std_logic_vector(31 downto 0);
		MEMIF_Mem2Hwt_Empty   : in  std_logic;
		MEMIF_Mem2Hwt_RE      : out std_logic;

		HWT_Clk    : in  std_logic;
		HWT_Rst    : in  std_logic;
		HWT_Signal : in  std_logic;
		
		DEBUG : out std_logic_vector(110 downto 0);
		
		-- AXIS Video in ports
		s_axis_video_tdata  : in  std_logic_vector(23 downto 0);
		s_axis_video_tvalid : in  std_logic;
		s_axis_video_tuser  : in  std_logic;
		s_axis_video_tlast  : in  std_logic;
		s_axis_video_tready : out std_logic;
		
		-- AXIS Video out ports
		m_axis_video_tdata  : out std_logic_vector(23 downto 0);
		m_axis_video_tvalid : out std_logic;
		m_axis_video_tuser  : out std_logic;
		m_axis_video_tlast  : out std_logic;
		m_axis_video_tready : in  std_logic
	);
end entity rt_reconf;

-- This is just a dummy module, no implementation required --

architecture dummy of rt_reconf is

begin
	
end architecture;