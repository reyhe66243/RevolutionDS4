.PHONY: all firmware cios clean

all:
	@./build.sh all

firmware:
	@./build.sh firmware

cios:
	@./build.sh cios

clean:
	@./build.sh clean
