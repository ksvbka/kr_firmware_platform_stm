CC=sdcc
AR=sdar

vpath %.c STM8S_StdPeriph_Driver/src

CFLAGS=-c -mstm8 -DSTM8S105 -ISTM8S_StdPeriph_Driver/inc --opt-code-size
LDFLAGS=-rc
SOURCES=stm8s_flash.c stm8s_uart2.c stm8s_tim4.c stm8s_clk.c stm8s_itc.c
# SOURCES=stm8s_minilib.c

OBJECTS=$(SOURCES:.c=.o)
OBJECTS_LINK=$(SOURCES:.c=.rel)
EXECUTABLE=stm8s_minilib.lib

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(AR) $(LDFLAGS) $(EXECUTABLE) $(OBJECTS_LINK)

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.lib
	rm -rf *.rel
	rm -rf *.lst
	rm -rf *.ihx
	rm -rf *.sym
	rm -rf *.asm
	rm -rf *.lk
	rm -rf *.map
	rm -rf $(EXECUTABLE)
	rm -rf *.rst
