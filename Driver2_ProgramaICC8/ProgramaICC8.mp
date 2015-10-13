Project:      ..\ProgramaICC8\ProgramaICC8
Build Number: 19
Date:         Sat Aug 15 21:37:58 2015
Compiler:     ICCAVR
Version:      8.04.01

NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   0026   0010 =     16. bytes (rel,con,rom,lit)

       Addr  Global Symbol
      -----  --------------------------------
       0026  __idata_start
       0036  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0036   046A =   1130. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       001B  __start
       001B  __text_start
       003C  _exit
       003D  _main
       00E6  _Config
       00FF  _ActualizaVel
       0154  _ADCCH
       0165  _SPI_SlaveTransfer
       0171  _delayms
       01A6  _int0_isr
       01A7  _int1_isr
       01A8  _timer2_comp_isr
       01A9  _timer2_ovf_isr
       01AA  _timer1_capt_isr
       01AB  _timer1_compa_isr
       01B4  _timer1_compb_isr
       01BD  _timer1_ovf_isr
       01E2  _timer0_ovf_isr
       01E3  _spi_stc_isr
       01E4  _uart0_rx_isr
       01E5  _uart0_udre_isr
       01E6  _uart0_tx_isr
       01E7  _adc_isr
       01E8  _eeprom_ready_isr
       01E9  _ana_comp_isr
       01EA  _twi_isr
       01EB  _spm_rdy_isr
       01EC  push_arg4
       01EE  push_arg2
       01F1  empy32u
       01F1  empy32s
       0221  push_xgsetF000
       0226  pop_xgsetF000
       022B  long_prolog
       0239  long_epilog
       0244  tstzero1
       024A  tstzero2
       0250  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0060   0010 =     16. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0060  _i
       0060  __data_start
       0062  _VM1
       0064  _VM2
       0066  _dM1
       0067  _dM2
       0068  _L
       006C  _Esclavo_ID
       006D  _Haciendo
       006E  _idx
       0070  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0070   0002 =      2. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0070  _DatoSPI
       0070  __bss_start
       0072  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   0026 =     38. bytes (abs,ovr,rom,code)

Files Linked      [ module(s) ]

C:\iccv8avr\lib\crtAVR.o	[ crtavr.s ]
.objs\main.o	[ main.c ]
<library>	[ asave.s, emul32.s, gpushy.s, longutil.s ]

User Global Definitions

ram_end = 0x45f
hwstk_size = 0x14

Area Definitions

-bfunc_lit:0x26.0x2000
-bdata:0x60.0x45f

