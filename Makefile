# Copyright 2010 Beoran. 
# 
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

include $(GOROOT)/src/Make.$(GOARCH)

all: libs test-tamias

libs:
	make -C tamias install

test-tamias: test-tamias.go libs
	$(GC) test-tamias.go
	$(LD) -o $@ test-tamias.$(O)

clean:
	make -C tamias clean
	rm -f -r *.8 *.6 *.o */*.8 */*.6 */*.o */_obj test-tamias
	