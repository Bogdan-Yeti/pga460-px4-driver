px4_add_module(
	MODULE drivers__pga460
	MAIN pga460
	STACK_MAIN 2048
	SRCS
		pga460.cpp
	DEPENDS
		px4_work_queue
	)
