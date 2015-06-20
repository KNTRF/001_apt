＜概要＞
FreeRTOS v8.2.1をベースに以下を修正

＜変更点＞
①オリジナルのFreeRTOSのフォルダ構成を整理
　＜/inc内のファイルの元位置＞
　/include
　
　＜/src内のファイルの元位置＞
　/*.c 6files
　/portable/GCC/ARM_CM3/port.c
　/portable/GCC/ARM_CM3/portmacro.c
　/portable/MemMang/heap2.c

②一部ファイルの修正。(ファイル位置関係)
　deprecated_definitions.h　201行目 #ifdef GCC_ARMCM3内

＜参考URL＞
・http://www.fumi2kick.com/komekame/archives/1172
