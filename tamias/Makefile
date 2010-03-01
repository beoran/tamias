# Copyright 2010 Beoran.  All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

include $(GOROOT)/src/Make.$(GOARCH)

TARG=tamias

# GOFILES:=constants.$(O).go

GOFILES:=tamias.go float.go bb.go body.go shape.go vect.go 

# CGO_CFLAGS:=-I/usr/local/include/opencv -I/usr/include/opencv

# CGO_LDFLAGS:=-L/usr/local/lib -lcxcore -lcv -lhighgui -lcallback

CLEANFILES+=tamias

include $(GOROOT)/src/Make.pkg
