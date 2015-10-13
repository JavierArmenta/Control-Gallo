CFLAGS =  -e -D__ICC_VERSION=80401 -D__BUILD=0 -DATMega8  -l -A -g -MHasMul -MEnhanced -Wa-W 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -cross_module_type_checking -g -nb:0 -e:0x2000 -Wl-W -bfunc_lit:0x26.0x2000 -dram_end:0x45f -bdata:0x60.0x45f -dhwstk_size:20 -beeprom:0.512 -fihx_coff -S2
FILES = main.o 

default:	$(FILES)
	$(CC) -o default $(LFLAGS) @..\ProgramaICC8\ProgramaICC8.lk  -lstudio -lcavr
