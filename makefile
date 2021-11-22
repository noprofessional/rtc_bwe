.PHONY:all

SRCS=remote_bitrate_estimator.cpp client.cpp inter_arrival.cpp overuse_detector.cpp overuse_estimator.cpp rate_statics.cpp
OBJS=$(SRCS:.cpp=.o)
DEPS=$(OBJS:.o=.d)
$(info $(OBJS))

all: client server

client: $(OBJS)
	g++ -o $@ $(OBJS)

%.o:%.cpp %.d
	g++ -c $< -Wall -Wextra -Werror -Wfatal-errors -MMD -MT $@ 

$(DEPS): ;

clean:
	rm -rf $(OBJS) $(DEPS) client server

server: server.cpp
	g++ -o server server.cpp -Wall -Wextra -Werror -Wfatal-errors

include $(wildcard $(DEPS))
