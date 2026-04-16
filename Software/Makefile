CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
SIZE = arm-none-eabi-size

CPPFLAGS = -mcpu=cortex-m4 -g3 -O0 -ffunction-sections -fdata-sections -Wall -Wextra --specs=nano.specs -mfloat-abi=soft -mthumb -I Inc -fstack-usage
CFLAGS = -std=gnu11
CXXFLAGS = -std=c++2a -fno-exceptions -fno-rtti -fno-use-cxa-atexit

AS = arm-none-eabi-gcc
ASFLAGS = -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp --specs=nano.specs -mfloat-abi=soft -mthumb

OBJECTS = $(addsuffix .o,$(basename $(wildcard Src/*.cpp Src/*.c Src/*.s)))

main.elf: $(OBJECTS)
	arm-none-eabi-g++ -o $@ $^ -mcpu=cortex-m4 -T STM32L431KCUX_FLASH.ld -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group

clean:
	rm $(wildcard Src/*o)
	rm $(wildcard ./main.elf)

####----####----####----####----####----####----####----####----####----####----####----####----####----####----####----####----####----####

stackusage: $(OBJECTS)
	cat $(addsuffix .su, $(basename $(wildcard Src/*cpp))) | awk 'BEGIN{FS="\t"} {print $$2 " " $$1 " " $$3;}' | sort -n -r | less

memoryusage: $(OBJECTS)
	$(SIZE) $^

all: clean main.elf
