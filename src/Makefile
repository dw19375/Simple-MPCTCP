# Makefile for smpctcp

CC = clang
CFLAGS  = -g -Wall


all: smpctcp

smpctcp: main.o smpctcp_cli.o smpctcp_srv.o pslist.o net_util.o coding_util.o util.o
	$(CC) $(CFLAGS) -o smpctcp main.o smpctcp_cli.o smpctcp_srv.o pslist.o net_util.o coding_util.o util.o 
	
main.o: main.c net_util.h util.h smpctcp_cli.o smpctcp_srv.o
	$(CC) $(CFLAGS) -c main.c
	
smpctcp_cli.o: smpctcp_cli.c smpctcp_cli.h
	$(CC) $(CFLAGS) -c smpctcp_cli.c

smpctcp_srv.o: smpctcp_srv.c smpctcp_srv.h
	$(CC) $(CFLAGS) -c smpctcp_srv.c
	
pslist.o: pslist.c pslist.h net_util.h
	$(CC) $(CFLAGS) -c pslist.c

net_util.o: net_util.c net_util.h util.h
	$(CC) $(CFLAGS) -c net_util.c

coding_util.o: coding_util.c coding_util.h net_util.h
	$(CC) $(CFLAGS) -c coding_util.c

util.o: util.c util.h net_util.h default_config.o
	$(CC) $(CFLAGS) -c util.c
	
default_config.o: default_config.c default_config.h
	$(CC) $(CFLAGS) -c default_config.c
	
clean:
	rm -f smpctcp *.o
