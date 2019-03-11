# find the mk/ directory, which is where this makefile fragment
# lives. (patsubst strips the trailing slash.)
SYSTYPE			:=	$(shell uname)#�жϱ���ϵͳ����
# ȷ����ǰmk�ļ�Ŀ¼������MK_DIR��
ifneq ($(findstring CYGWIN, $(SYSTYPE)),) 
  MK_DIR := $(shell cygpath -m ../mk)
else
  MK_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
endif

#include �����������ļ������α���
include $(MK_DIR)/environ.mk#���û�������

# short-circuit build for the configure target
ifeq ($(MAKECMDGOALS),configure)
include $(MK_DIR)/configure.mk

else

#short-circuit build for the help target
include $(MK_DIR)/help.mk

# common makefile components
include $(MK_DIR)/targets.mk
include $(MK_DIR)/sketch_sources.mk
include $(SKETCHBOOK)/modules/uavcan/libuavcan/include.mk

ifneq ($(MAKECMDGOALS),clean)
#�ɴ�ѡ����ڲ�ͬ�İ汾�Ĵ���
# board specific includes
ifeq ($(HAL_BOARD),HAL_BOARD_SITL)
include $(MK_DIR)/board_native.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_LINUX)
include $(MK_DIR)/board_linux.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_PX4)
include $(MK_DIR)/board_px4.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_VRBRAIN)
include $(MK_DIR)/board_vrbrain.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_F4LIGHT)
include $(MK_DIR)/board_F4Light.mk
endif

endif

endif
