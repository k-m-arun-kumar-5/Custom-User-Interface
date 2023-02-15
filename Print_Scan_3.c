/* ********************************************************************
FILE                   : Print_Scan_2.c

PROGRAM DESCRIPTION    : implementation of scanf and printf function

AUTHOR                :  K.M. Arun Kumar alias Arunkumar Murugeswaran
	 
KNOWN BUGS            :    

NOTE                  :  Compiled and Tested in Dev-C++ on Windows 7 (32 bit) Desktop OS.
                         float point precision data is more accurate than in Print_Scan_2.c

reference             : 

   https://www.menie.org/georges/embedded/small_printf_source_code.html
   https://www.geeksforgeeks.org/program-for-conversion-of-32-bits-single-precision-ieee-754-floating-point-representation/ 
   https://www.geeksforgeeks.org/convert-floating-point-number-string/
   http://www.ryanjuckett.com/programming/printing-floating-point-numbers/ 
   http://mirror.fsf.org/pmon2000/2.x/src/lib/libc/scanf.c
   https://opensource.apple.com/source/xnu/xnu-1228/libkern/stdio/scanf.c
                                    
CHANGE LOGS           : 

*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <stdarg.h>

//#define TRACE                                 (1U)
#define TRACE_ERROR                             (2U)
#define TRACE_REQ                               (3U)
#define TRACE_INFO                              (4U)
#define TRACE_DATA                              (5U)
#define TRACE_FLOW                              (6U)

#define NULL_DATA_PTR                                      ((void *)0)
#define STATE_NO                                 (0)
#define STATE_YES                                (1)
#define STATE_NA                                 (2) 


#define PAD_RIGHT                     (1 << 0)
#define PAD_ZERO                      (1 << 1)

#define LITTLE_ENDIAN() ((*((const char *) &endian_test_data) == 0x01))

#define CHARBITS               (8)
#define FLOATBITS              (sizeof(float) * CHARBITS)

/* extract nth LSB from object stored in lvalue x */
#define GET_BIT(float_num, float_bit_index) ((((const char *) &float_num)[LITTLE_ENDIAN() ? (float_bit_index) / CHARBITS : sizeof(float_num) - (float_bit_index) / CHARBITS - 1] >> ((float_bit_index) % CHARBITS)) & 0x01)

#define PUT_BIT(conv_float, float_bit_index) ((GET_BIT((conv_float.raw_ieee_754_format.float_num), (float_bit_index)) ? (conv_float.ieee_754_format |= (1 << float_bit_index)) : (conv_float.ieee_754_format &= ~(1 << float_bit_index)) ))

/*
 * Flags used during scanf conversion.
 */
#define	LONG		       (0x01)	/* l: long or double */
#define	SHORT		       (0x04)	/* h: short */
#define	SUPPRESS	       (0x08)	/* *: suppress assignment */
#define	POINTER		       (0x10)	/* p: void * (as hex) */
#define	NOSKIP		       (0x20)	/* [ or c: do not skip blanks */
#define	LONGLONG	      (0x400)	/* ll: long long (+ deprecated q: quad) */
#define	SHORTSHORT	     (0x4000)	/* hh: char */
#define	UNSIGNED	     (0x8000)	/* %[oupxX] conversions */
#define REAL            (0x10000)   /* float or double */
/*
 * The following are used in numeric conversions only:
 * SIGNOK, NDIGITS, DPTOK, and EXPOK are for floating point;
 * SIGNOK, NDIGITS, PFXOK, and NZDIGITS are for integral.
 */
#define	SIGNOK	    	(0x40)	/* +/- is (still) legal */
#define	NDIGITS		    (0x80)	/* no digits detected */

#define	DPTOK		   (0x100)	/* (float) decimal point is still legal */
#define	EXPOK		   (0x200)	/* (float) exponent (e+3, etc) still legal */

#define	PFXOK		  (0x100)	/* 0x prefix is (still) legal */
#define	NZDIGITS	  (0x200)	/* no zero digits detected */

/*
 * scanf Conversion types.
 */
#define	CT_CHAR		(0)	/* %c conversion */
#define	CT_CCL		(1)	/* %[...] conversion */
#define	CT_STRING	(2)	/* %s conversion */
#define	CT_INT		(3)	/* %[dioupxX] conversion */
#define CT_REAL     (4)   /* [%f or %lf */
#define	UQUAD_MAX	((u_quad_t)0-1)	/* max value for a uquad_t */
					/* max value for a quad_t */
#define	QUAD_MAX	((quad_t)(UQUAD_MAX >> 1))
#define	QUAD_MIN	(-QUAD_MAX-1)	/* min value for a quad_t */

#define SUCCESS                                 (0)
#define FAILURE                                 (1)
 
#define BACKSPACE_CHAR                        ('\b')
#define ENTER_CHAR                            ('\n')
#define NEW_LINE_CHAR                         ('\n')
#define HORIZONTAL_TAB                        ('\t')
#define VERTICAL_TAB                          ('\v')
#define NULL_CHAR                             ('\0')   
#define NUM_0_CHAR                            ('0')
#define NUM_9_CHAR                            ('9')
#define ENGLISH_SMALL_ALPHA_a_CHAR            ('a')
#define ENGLISH_SMALL_ALPHA_z_CHAR            ('z')
#define ENGLISH_BIG_ALPHA_A_CHAR              ('A')
#define ENGLISH_BIG_ALPHA_Z_CHAR              ('Z')
#define BEGIN_CTRL_CHARS_ASCII_CODE           ('\x0')
#define END_CTRL_CHARS_ASCII_CODE             ('\x1F')                                  
#define ASCII_MAX_CHAR_CODE                   ('\x7F')

#define UART_MOD_ENABLE                       (1)
#define KEYBOARD_MOD_ENABLE                   (2)

#define MAX_STREAM_BUFFER_SIZE                       (50)
/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN                                (12)

#define MAX_WIDTH_SPECIFIER                         (PRINT_BUF_LEN - 1)     
#define MAX_FLOAT_PREC_SPECIFIER                          (6)
#define DEFAULT_FLOAT_PREC_WIDTH                          (6)

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long uint64_t;

typedef char int8_t;
typedef short int int16_t;
typedef int int32_t;
typedef long int64_t;

typedef unsigned int u_quad_t;
typedef int quad_t;
typedef long double real_t;
typedef void *     ptr_t;

typedef enum
{
	BASE_02 = 2, BASE_08 = 8, BASE_10 = 10, BASE_16 = 16
} base_t;

typedef enum 
{
	NUM_CONV_BIG_ALPHA, NUM_CONV_SMALL_ALPHA, NUM_CONV_ALPHA_NA
} num_conv_alpha_t;

typedef enum 
{
	GET_NUM_DIGITS, GET_POWER_OF_CUR_NUM_DIGITS, GET_POWER_OF_NEXT_NUM_DIGITS  
} get_num_digits_type_t;

typedef enum
{
	BASE_BIT_POS = 0, SIGN_FLAG_BIT_POS = 5, PAD_FORMAT_BIT_POS = 6, ALPHA_BIT_POS = 8, REAL_NUM_TYPE_BIT_POS = 10
} print_num_flag_bit_pos_t;

typedef union
{ 
  
    float float_num; 
    struct
    { 
        // Order is important. 
        // Here the members of the union data structure 
        // use the same memory (32 bits). 
        // The ordering is taken 
        // from the LSB to the MSB. 
        uint32_t mantissa  : 23; 
        uint32_t exponient : 8; 
        uint32_t sign      : 1; 
    } raw; 
} raw_ieee_754_format_t; 

typedef struct 
{
	raw_ieee_754_format_t raw_ieee_754_format;	
	uint32_t ieee_754_format;	
} float_ieee_754_format_t;	

typedef enum 
{
	REAL_VAL_IS_NEGATIVE_INFINITY, REAL_VAL_IS_POSITIVE_INFINITY,  REAL_VAL_IS_NEGATIVE_NOT_A_NUM, REAL_VAL_IS_POSITIVE_NOT_A_NUM, REAL_VAL_IS_NEGATIVE_ZERO,
	REAL_VAL_IS_POSITIVE_ZERO, REAL_VAL_IS_NEGATIVE_DENORMALISED, REAL_VAL_IS_POSITIVE_DENORMALISED, REAL_VAL_IS_NEGATIVE_NORMALISED, REAL_VAL_IS_POSITIVE_NORMALISED 	
} ieee_754_real_val_data_status;

typedef union 
{
	long double long_double_num;
	double double_num;
	float float_num;
} real_num_t;

typedef enum 
{
	FLOAT_FIXED_POINT_TYPE, DOUBLE_FIXED_POINT_TYPE, LONG_DOUBLE_FIXED_POINT_TYPE
} real_num_type_t; 
	
typedef enum
{
	CHAR_DEVICE, BLOCK_DEVICE, DEVICE_NA
} device_data_type_t;

typedef enum 
{
	STREAM_IO_INPUT, STREAM_IO_OUTPUT, STREAM_IO_BOTH, STREAM_IO_NA
} stream_io_type_t;

typedef enum 
{
	STREAM_TYPE_IO, STREAM_TYPE_FILE, STREAM_TYPE_SOCKET, STREAM_TYPE_NA
} stream_type_t ;

typedef struct file_t
{
	char stream_buf[MAX_STREAM_BUFFER_SIZE + 1];
	uint8_t file_desp;
	uint8_t num_chars_stream_buf;
	uint8_t stream_buf_cur_pos;
	uint8_t device_type	    : 2; //char or block
	uint8_t stream_type     : 2; //io_type, file or socket
	uint8_t stream_io_type  : 2; //input, output, both	
	uint8_t                 : 2;	
} file_t;

typedef enum 
{ 
    STDIN_FILE_DESP, STDOUT_FILE_DESP, STDERR_FILE_DESP, STR_FILE_DESP  
} file_desp_pre_conf_t;

typedef enum 
{
	NO_ERROR, ERR_IO_CONFIG_NULL_PTR, ERR_PORT_WRITE_VAL, ERR_PORT_WRITE_INIT_VAL, ERR_IO_CH_WRITE_DATA,
	ERR_IO_CONFIG_INDEX_INVALID, ERR_IO_CH_INVALID, ERR_CONFIG_IO_CH_MATCH_INVALID, ERR_GPIO_FUNC_SET,  ERR_IO_CH_00_FUNC_SET, 
	ERR_TRACE_FUNC_SET, ERR_IO_CH_48_FUNC_SET, ERR_IO_CH_58_FUNC_SET, ERR_PORT1_PIN_FUNC_SET, ERR_PORT_INVALID,
    ERR_CONFIG_PORT_INIT_VAL, ERR_IO_CH_GPIO_FUNC_SET, ERR_SW_CH_GPIO_FUNC_SET, ERR_GPIO_INPUT_FUNC_STATE, ERR_PIN_SIGNAL,
	ERR_PORT0_IO_PIN, ERR_PORT1_IO_PIN,  ERR_GPIO_CH_SET_PORT, ERR_IO_PIN_RANGE,  ERR_IO_CH_24_PIN, ERR_IO_CH_32_TO_47,
	ERR_CONF_INDEX_IO_CH_NULL_PTR, ERR_PORT_TO_IO_CONF_NULL_PTR, ERR_MAX_SW_CH_EXCEEDS, ERR_CONSUCC_PARA, ERR_STR_TO_NUM_PARA,
	ERR_STR_PTR_NULL, ERR_GPIO_OUTPUT_DATA, ERR_INVALID_PORT, ERR_IO_CH_READ, ERR_IO_CH_TO_SW_CH,
	ERR_IO_CH_RESERVE_PIN_FUNC_SET, ERR_GPIO_OUTPUT_FUNC_STATE, ERR_SW_CH_NOT_MATCH_IO_CH, ERR_CONFIG_PIN_RANGE, 
	ERR_CONFIG_DEBUG_FUNC_SET, ERR_CONFIG_TRACE_FUNC_SET, ERR_IO_CONFIG_TABLE_LARGE, ERR_TEST_FAIL_1_CONSUCC_BITS, ERR_TEST_FAIL_0_CONSUCC_BITS,
	WARN_CUR_DATA_ID_DEV_DISABLED, ERR_KEYPAD_ROW_WRITE, ERR_KEYPAD_INVALID_ROW, ERR_KEYPAD_COL_INPUT, ERR_KEYPAD_NO_ACCESS_COL,
	ERR_CUR_DATA_ID_READ_EXCEEDS_MAX_CHARS, ERR_SW_READ_PROC, ERR_SW_NO_ACCESS, ERR_READ_PORT_VAL, ERR_SEG7_NO_WRITE,
	SW_OR_KEY_NOT_PRESSED, CUR_DATA_ID_UNTIL_LAST_TRY_COMPLETE_READ_AND_READY_PROCESS, CUR_DATA_ID_REACH_MAX_CHARS_READ_WAIT_TERMINATOR_CHAR, CUR_DATA_ID_NUM_CHAR_READ, WARN_CUR_DATA_ID_SPECIAL_CHAR_READ_BUT_DISABLED, 
	ERR_KEYBOARD_READ_OPER, ERR_IO_CH_WRITE_NON_WRITE, ERR_PORT_WRITE_BIT_VAL, ERR_DATA_ID_EXCEED, ERR_NULL_PTR,
	ERR_FORMAT_INVALID, ERR_EXCEEDS_DATA_NUM_CHARS, ERR_DEV_SRC_ID_INVALID, ERR_ALREADY_MEM_ALLOC ,ERR_DATA_ID_CONF,
	ERR_DEV_CH_ID_EXCEEDS, ERR_DEV_CH_ID_OR_NULL_PTR, WARN_CUR_DATA_ID_DEV_NO_ACCESS, WARN_PREV_AND_CUR_DATA_MATCH, 
	ERR_SEG7_DISABLED, CUR_DATA_ID_SPECIAL_CHAR_READ, ERR_DEV_INIT, ERR_DEV_ALLOW_ACCESS, ERR_IO_CH_UNCONFIG,
	ERR_IO_CH_FUNC_RESET, ERR_DEV_OPER_DEINIT_FUNC, ERR_DEV_OPER_INIT_FUNC, ERR_DEV_OPER_NO_ACCESS_FUNC, ERR_DEV_OPER_ALLOW_ACCESS_FUNC, 
	ERR_DEV_OPER_READ_FUNC, ERR_DEV_OPER_OUTPUT_FUNC, CUR_DATA_ID_SMALL_ENGLISH_ALPHA_CHAR_READ, CUR_DATA_ID_BIG_ENGLISH_ALPHA_CHAR_READ, WARN_CUR_DATA_ID_NUM_CHAR_READ_BUT_DISABLED,
	WARN_CUR_DATA_ID_SMALL_ENGLISH_ALPHA_CHAR_READ_BUT_DISABLED, WARN_CUR_DATA_ID_BIG_ENGLISH_ALPHA_CHAR_READ_BUT_DISABLED, DATA_CHAR_READ, WARN_CUR_DATA_ID_CTRL_CHAR_READ_BUT_DISABLED, ERR_INVALID_DATA_CHAR_READ,
	CUR_DATA_ID_CTRL_CHAR_INPUT, CUR_DATA_ID_VALID_CHAR_READ, ERR_DEV_OPER_DISABLE_FUNC, ERR_DEV_OPER_ENABLE_FUNC, ERR_NO_INPUT_DEV, 
	ERR_INPUT_DEV_ID_OPER_FUNC, ERR_DEV_OPER_OPEN_FUNC, ERR_DEV_OPER_CLOSE_FUNC, ERR_NO_OUTPUT_DEV, ERR_INVALID_OUTPUT_DEV_OPER_FUNC, 
	ERR_INPUT_DEV_CH_ID_NOT_MATCH, ERR_OUTPUT_DEV_CH_ID_NOT_MATCH, ERR_DATA_TYPE_INVALID, ERR_NO_COMM_DEV, ERR_DEV_SRC_CH_ID_AND_FILTER_NOT_MATCH,
	ERR_DEV_SRC_BASIC_OPER_FUNC_INVALID, ERR_DEV_ID_CONF, ERR_CUR_DATA_ID_INVALID, ERR_CUR_DATA_DEV_SRC_ALLOW_NOT_SRC_LIST, ERR_LED_MATRIX_WRITE,
	ERR_SW_CONFIG, ERR_SW_CONFIG_FIXED_NOT_ALLOC, ERR_SW_CONFIG_DYNAMIC_NOT_ALLOC, ERR_LCD_WRITE_PROC, ERR_LCD_READ_PROC,
	ERR_LCD_INVALID_LOC, ERR_LCD_CONFIG_INVALID, ERR_IO_CH_READ_NON_READ, ERR_TEST_FAIL_VAL_CONSUCC_BITS, ERR_UART_INIT, 
	ERR_CHECK_GPIO_FUNC, ERR_NON_GPIO_FUNC_SET, ERR_COMM_TX_PROC, ERR_COMM_RCV_PROC, COMM_NO_RCV_CHAR, 
	COMM_RCVD_CHAR, ERR_DEV_OPER_RECEIVE_FUNC, ERR_LCD_DISP_FORMAT, ERR_UART_DISP_FORMAT, WARN_LCD_ALLOC_LESS_THAN_CONFIG_CHARS,
	ERR_LED_MATRIX_DATA_TYPE_INVALID, ERR_DEV_OPER_WRITE_FUNC, ERR_PROC_ERROR, WARN_LCD_TRY_CHAR_ATEND_CONF_LINE, WARN_LCD_TRY_BS_CHAR_ATBEGIN,
	VALID_BS_CHAR_READ, ERR_UART_OVERFLOW_RUN, ERR_UART_FRAMING, ERR_DEV_ID_INVALID, ERR_DEV_ID_NOT_MATCH,
	ERR_CUR_DATA_ID_INVALID_DATA_TYPE, ERR_UART_NUM_DISP, ERR_NEXT_DATA_CONF, ERR_LCD_DISP_GOTO_XY, ERR_LCD_INPUT_GOTO_XY,
	ERR_LCD_WRITE_PROC_OPER, ERR_APPL_DATA_RETRIEVE, ERR_LCD_GOTO_XY_PROC, ERR_UART_TX_PROC, ERR_UART_RCV_PROC,
	 ERR_KEYBOARD_ENTER_SW_OPER_BUT_NOT_CONFIG, ERR_KEYBOARD_BS_SW_OPER_BUT_NOT_CONFIG, ERR_KEYBOARD_PART_OPER, ERR_SW_PRESS_PROC, ERR_DEV_OPER_FUNC_PROC,
	 ERR_KEYBOARD_ENTER_SW_PROC, ERR_KEYBOARD_BS_SW_PROC, ERR_KEYBOARD_COL_SCAN_PROC, ERR_INVALID_DATA, VALID_ENTER_CHAR_READ, 
	 ERR_RESET_OPER, ERR_SW_NOT_ALLOC, ERR_SW_CH_INVALID, VALID_CTRL_CHAR_READ, ERR_HW_INIT,
	 CUR_DATA_ID_READ_MAX_TRIED, CUR_DATA_ID_NO_INPUT_DEV, ERR_COMM_DEV_ID_OPER_FUNC, ERR_OUTPUT_DEV_ID_OPER_FUNC, ERR_RESET_DATA_ID_STATUS,
	 CUR_DATA_ID_LAST_TRY_COMPLETE_READ_AND_READY_PROCESS, CUR_DATA_ID_COMPLETE_READ_AND_READY_PROCESS, ERR_UART_PARITY, WARN_LCD_END_POINT_REACHED, ERR_UART_DEINIT,
	 WARN_TRANSMIT_END_POINT_REACHED, WARN_RECEIVE_END_POINT_REACHED, CUR_DATA_ID_NO_RECEIVER_DEV, ERR_READ_DEV_TYPE, ERR_WRITE_DEV_TYPE,
	 CUR_DATA_ID_NO_OUTPUT_DEV, CUR_DATA_ID_NO_TRANSMIT_DEV, CUR_DATA_ID_NO_STREAM, ERR_PRINTF_FLAG_INVALID, ERR_CUR_DATA_DEV_NOT_SRC, 
	 ERR_UART_TRANSMIT_OPER, ERR_STR_LEN_OPER, ERR_KEYBOARD_READY_READ, ERR_TIMER_ID_EXCEEDS, ERR_IO_CH_WRITE,
	 ERR_TIMER_RUN_PROC, ERR_TIMER_STOP_PROC, ERR_IRQ_VECTOR_SLOT, ERR_EXT_INTERRUPT_ENABLE_PROC, ERR_ENABLE_INTERRUPT,
	 ERR_INTERRUPT_SRCS_INVALID, ERR_TIMEOUT_PROC, ERR_EXT_INTERRUPT_PROC, ERR_IRQ_VECTOR_ALREADY_ALLOC, ERR_UART_SET_STATUS_PROC,
	 ERR_UNSUPPORTED, ERR_ALREADY_ENABLED_INTERRUPT_SRC, ERR_DISABLE_INTERRUPT, ERR_INTP_PROCESS_TO_EXEC_EXCEEDS, ERR_SRC_INTP_DATA_ARR_FULL,
	 TMR_NO_MAX_NUM_TIMEOUT_YET_PROCESS, TMR_AT_LAST_TIMEOUT_YET_PROCESS, TMR_BEFORE_LAST_TIMEOUT_YET_PROCESS, TMR_MAX_TIMEOUT_YET_PROCESS, NO_NEED_PROCESS_INTERRUPTED,
	 ERR_RESET_SRC_INTP_DATA_ARR_PROC, ERR_PROCESS_INTP_STATUS_FLAG_INVALID, SEARCH_INTP_SRC_DATA_ARR_NOT_FOUND, SEARCH_INTP_SRC_ONE_DATA_ARR, SEARCH_INTP_SRC_MORE_THAN_ONE_DATA_ARR,
	 EXT_INTP_OCCURRED_YET_PROCESS,	ERR_SEARCH_INTP_SRC_DATA_PROC, ERR_DELETE_INTP_SRC_DATA_PROC, TMR_NO_MAX_NUM_TIMEOUT_PROC, TMR_MAX_NUM_TIMEOUT_PROC,
	 TMR_BEFORE_LAST_TIMEOUT_PROC, TMR_AT_LAST_TIMEOUT_PROC, ERR_TMR_TIMEOUT_TYPE_INVALID, ERR_RETRIEVE_INTP_SRC_DATA_PROC, ERR_DEV_ALLOW_ACCESS_PROC,
	 ERR_DEV_ENABLE_PROC, ERR_DEV_DISABLE_PROC, ERR_TMR_ID_INVALID, ERR_SW_PRESENT_STATE_PROC, ERR_SW_DEBOUNCE, 
	 ERR_TIMER_ALREADY_STOPPED_IN_PAUSE_PROC, ERR_TIMER_TIMEOUT_LESS_THAN_MIN, ERR_TIMER_NOT_IN_PAUSE_STATE_IN_RESUME, ERR_TIMER_PAUSE_PROC, ERR_TIMER_RESUME_PROC, 
	 ERR_UART_RETRIEVE_DATA_ARR_PROC, UART_OVERFLOW_LINE_STATUS, UART_PARITY_ERROR_LINE_STATUS, UART_FRAMING_ERROR_LINE_STATUS, UART_RECEIVER_IDLE_LINE_STATUS, 
	 UART_RECEIVE_DATA_AVAIL, UART_RECEIVE_CHAR_TIMEOUT, UART_TRANSMIT_REG_EMPTY, ERR_UART_PROC, ERR_UART_LINE_STATUS_PROC, 
	 ERR_QUEUE_FULL, ERR_QUEUE_EMPTY, ERR_ENQUEUE_PROC, ERR_DEQUEUE_PROC, ERR_QUEUE_INSERT_FRONT_PROC, 
	 ERR_QUEUE_DELETE_REAR_PROC, ERR_ADC_BURST_MODE_BUT_START_MODE_INVALID, ERR_ADC_FUNC_SET_PROC, ERR_ADC_SW_CTRL_MODE_SEL_CHS_MORE_THAN_ONE, ERR_ADC_SW_CTRL_BUT_RESULT_BITS_INVALID,
	 ERR_ADC_NON_SEL_CH_BUT_TO_INTP, ERR_IO_PIN_NOT_CONFIG, ERR_ADC_GLOBAL_INTP_BUT_SEL_ENABLED, ERR_ADC_ENABLE_INTERRUPT_BUT_NOT_CONF, ERR_ADC_STATUS_PROC, 
	 ERR_ADC_INIT_PROC, WARN_ADC_IS_CONVERTING, WARN_ADC_IS_OVER_RUN, ERR_ADC_NOT_INIT_FOR_START_CONV, ERR_EXT_INTERRUPT_INIT, 
	 ERR_ADC_STOP_CONV_PROC, ERR_ADC_START_CONV_PROC, ERR_APPL_RESET_PROC, ERR_INIT_IO_FILE_PROC, ERR_DATA_OUT_OF_RANGE,
	 ERR_GET_NUM_DIGITS_DATA_PROC, ERR_EOF, ERR_STR_REVERSE_PROC, ERR_DATA_CONV_PROC, ERR_MOD_NOT_ENABLED, 
	 ERR_FILE_FLUSH_PROC, ERR_FILE_INIT_PROC, NUM_SYS_STATUS 
} system_status_flags_t;

typedef enum 
{
 	NO_ERROR_OCCURED, WARNING_OCCURED, ERROR_OCCURED, FATAL_OCCURED 
} proc_occured_status_t;

uint32_t system_status_flag = NO_ERROR;
const uint32_t endian_test_data = 0x04030201;
file_t stdin_keyboard, stdout_vt;

int16_t VFile_Scan(file_t *const fp, const char *const fmt, va_list *const scan_arg_list_ptr);
int16_t VFile_Print(file_t *const fp, const char *const fmt, va_list *const print_va_list_ptr);
static int16_t Scan_Data(const char *inp, const char  *const fmt0, va_list *const scan_va_list_ptr);
static int16_t Print_Data(va_list *const print_va_list_ptr, file_t *const out_file_ptr, const char *const format_ptr);
static char Print_Char(char **const out_str_ptr, const char print_char);
static int16_t Print_Str(char **const out_str_ptr, const char *print_str, const uint8_t conf_width_spec, const uint8_t pad_format);
static int16_t Print_Num(char **const out_str_ptr, const int32_t print_num, const uint8_t conf_width_spec, const uint16_t ctrl_flag );
static int16_t Print_Float(char **const out_str_ptr, const real_num_t print_real_num, const uint8_t conf_width_spec, const uint8_t conf_num_digits_after_point, const uint16_t ctrl_flag );
uint16_t Real_Val_To_IEEE_754_Float(const float src_float_num, float_ieee_754_format_t *const result_float_ptr);
uint32_t IEEE_754_Float_Raw_To_Int(const uint32_t ieee_754_float_format, const uint8_t low, const uint8_t high);
uint16_t IEEE_754_Float_To_Real_Val(const float_ieee_754_format_t *const src_float_ptr, float *const result_float_num_ptr);
uint16_t IEEE_754_Float_Data_Status(const float_ieee_754_format_t float_ieee_754_format, uint8_t *const float_data_status_ptr);
int16_t Int_To_Str(const int32_t num, char *const str_ptr, const uint8_t req_num_digits); 
uint16_t Real_Num_To_Str_Conv(char *const real_num_to_str_ptr, const real_num_t real_num, const uint8_t real_num_type, uint8_t num_digits_after_point); 
uint16_t Str_To_Real_Num_Conv(real_num_t *const real_num_ptr, const char *const para_real_num_in_str_ptr, const uint8_t real_num_type, uint8_t num_digits_after_point); 
int16_t File_Get_Char(file_t *const fp);
int16_t Read_Oper(file_t *const fp, char *const out_buf_ptr, const uint8_t max_num_chars_to_read);
char *File_Get_Str(char *const dst_str_ptr, file_t *const fp, const uint8_t max_chars_to_get);
int16_t File_Scan(file_t *const fp, const char *const fmt, ...);
int16_t Str_Scan(const char *const ibuf, const char *const fmt, ...);
int16_t File_Put_Char(const char put_char, file_t *const fp);
int16_t Write_Oper(file_t *const fp, const char *const in_buf_ptr, const uint8_t max_num_chars_to_write);
int16_t File_Put_Str(const char *const io_buf_ptr, file_t *const fp);
int16_t File_Print(file_t *const fp, const char *const format_ptr, ...);
int16_t Str_Print(char *out_str, const char *const format_ptr, ...);

static int16_t Stream_Stdin_Oper(const uint8_t keyboard_ch_id);
char Get_Char(void);
char *Get_Str(char *const get_str);
int16_t Scan(const char *const format_str,...);
char Put_Char(const char to_disp_char);
int16_t Put_Str(const char *const to_disp_str);
int16_t Print(const char* const format_ptr,...);
uint8_t Error_or_Warning_Proc(const char *const error_trace_str, const uint8_t warn_or_error_format, const uint32_t warning_or_error_code);
uint16_t Reverse_Str_Use_Same_Str(char *const str, const uint8_t len); 
double Power_Of(const uint8_t base, const int16_t power);
uint16_t Init_File(file_t *const file_ptr, const uint8_t file_desp, const uint8_t device_type, const uint8_t stream_type, const uint8_t stream_io_type);
uint16_t File_Flush(file_t *const file_ptr);
static const char *Scan_Input_Limit_Chars(char *tab, const char *fmt);
unsigned long strtoul(const char *const nptr, char **endptr, uint8_t base);
quad_t strtoq(const char *const nptr, char **endptr, uint8_t base);
uint16_t Char_Is_ASCII(const unsigned char test_char);
uint16_t Get_Num_Digits_Based_Data(const uint32_t num, uint32_t *const num_digits_data_ptr, const uint8_t base, const uint8_t get_num_digits_type);

/*------------------------------------------------------------*
FUNCTION NAME  : main

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 01.01 

Bugs           :  
-*------------------------------------------------------------*/
int main(void)
{
	float float_num = -20.56, disp_float_num;
	real_num_t real_num;
	char str_print[30];
    uint32_t uint32_num = 30, num_digits_based_data;
	int32_t int32_num = -40;
	uint16_t num_chars_printed = 0;
	int16_t int16_num = -20;
	uint8_t uint8_num = 97;	
    
	Init_File(&stdin_keyboard, STDIN_FILE_DESP, CHAR_DEVICE, STREAM_TYPE_IO, STREAM_IO_INPUT );
	Init_File(&stdout_vt, STDOUT_FILE_DESP, CHAR_DEVICE, STREAM_TYPE_IO, STREAM_IO_OUTPUT );
            
    Print("Print: float_num = %09.9f, ", float_num); 
	Print("Print: uint32_num = %-4X, %% hello \nPrint: Enter a float & num : ", uint32_num); 
	int16_num = Scan("%f %d", &float_num, &int32_num);
    Print("\nScan: int16_num = %d, \n Entered float = %10.9f & num = %d \n Enter a 2 numbers : ",int16_num, float_num, int32_num);    
    Scan("%d %d",&int32_num, &uint32_num);	
    Print("\nScan: Entered num1 = %d num2 = %d \n", int32_num , uint32_num); 
    strcpy(str_print, "-38.4763 35"); 
    Str_Scan(str_print, "%f %d", &float_num , &uint8_num);
    printf("printf: Entered float_num : %f \n", float_num);	
    Print("Scan: Entered float = %.9f, num = %d \n",float_num , uint8_num);
	Print("Enter a float = ");
	Scan("%s", str_print);
	if((Str_To_Real_Num_Conv(&real_num, str_print, FLOAT_FIXED_POINT_TYPE, DEFAULT_FLOAT_PREC_WIDTH)) != SUCCESS)
	{
		return FAILURE;
	}		
	printf("Conv: Entered float_str = %s to float_num = %f\n", str_print, real_num.float_num);
	Real_Num_To_Str_Conv(str_print, real_num, FLOAT_FIXED_POINT_TYPE, DEFAULT_FLOAT_PREC_WIDTH);
	printf("Conv: Entered float_num = %f to float_str = %s\n", real_num.float_num, str_print);
	Print("Enter a num = ");
	Scan("%u", &uint32_num);
	Get_Num_Digits_Based_Data(uint32_num, &num_digits_based_data, BASE_10, GET_NUM_DIGITS);
	Print("Print: Entered num = %u, num digits = %u\n", uint32_num, num_digits_based_data);
	Print("Enter a float and num : ");
	Scan("%f %d", &float_num, &int32_num);
	printf("printf: entered float : %f \n", float_num);
	Print("Print: Entered float num: %f & num = %d \n", float_num, int32_num); 
    return SUCCESS; 
}

/*------------------------------------------------------------*
FUNCTION NAME  : Real_Val_To_IEEE_754_Float

DESCRIPTION    : convert a real value to IEEE 754 floating point representaion 
								
INPUT          : 

OUTPUT         : 

NOTE           : single precision IEEE 754 floating point rep

Func ID        : 15.01 

Bugs           :  
-*------------------------------------------------------------*/
uint16_t Real_Val_To_IEEE_754_Float(const float src_float_num, float_ieee_754_format_t *const result_float_ptr)
{
	int8_t float_bit_index;
     
	 memset(result_float_ptr, NULL_CHAR, 1);
    if(result_float_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.01.01", ERROR_OCCURED, system_status_flag); 
		return system_status_flag; 
	}
	result_float_ptr->raw_ieee_754_format.float_num = src_float_num;
	result_float_ptr->ieee_754_format = 0;
    float_bit_index = FLOATBITS - 1;
	//sign_bit part
    PUT_BIT((*result_float_ptr), float_bit_index);
    result_float_ptr->raw_ieee_754_format.raw.sign  = (result_float_ptr->ieee_754_format >> 31) & 0x01;
   // exponient part
    for(float_bit_index--; float_bit_index >= 23; float_bit_index--)
	{
        PUT_BIT((*result_float_ptr), float_bit_index);
    }
	result_float_ptr->raw_ieee_754_format.raw.exponient = (result_float_ptr->ieee_754_format >> 23) & 0xFF;
	//mantissa part
    for(; float_bit_index  >= 0; float_bit_index--)
	{
        PUT_BIT((*result_float_ptr), float_bit_index);
    }
	result_float_ptr->raw_ieee_754_format.raw.mantissa = (result_float_ptr->ieee_754_format) & 0x7FFFFF;
	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : IEEE_754_Float_Raw_To_Int

DESCRIPTION    : convert a raw IEEE float rep to the corresponding integer
								
INPUT          : 

OUTPUT         : 

NOTE           : single precision IEEE 754 floating point rep

Func ID        : 15.02 

Bugs           :  
-*------------------------------------------------------------*/
uint32_t IEEE_754_Float_Raw_To_Int(const uint32_t ieee_754_float_format, const uint8_t low, const uint8_t high) 
{ 
    uint32_t float_to_int = 0, i; 
	
    for (i = high; i >= low; i--)
	{ 
        float_to_int = float_to_int + ((ieee_754_float_format >> (31 - i)) & 0x01) * (uint32_t)Power_Of(BASE_02, high - i); 
    } 
    return float_to_int; 
} 

/*------------------------------------------------------------*
FUNCTION NAME  : IEEE_754_Float_To_Real_Val

DESCRIPTION    : convert IEEE 754 floating point representation into real value 
								
INPUT          : 

OUTPUT         : 

NOTE           : single precision IEEE 754 floating point rep

Func ID        : 15.03 

Bugs           :  
-*------------------------------------------------------------*/
uint16_t IEEE_754_Float_To_Real_Val(const float_ieee_754_format_t *const src_float_ptr, float *const result_float_num_ptr)
{
	float_ieee_754_format_t result_float;
	uint32_t float_in_int;
	
	if(src_float_ptr == NULL_DATA_PTR || result_float_num_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.03.01", ERROR_OCCURED, system_status_flag); 
		return system_status_flag;
	}
    // Convert mantissa part (23 bits) to corresponding decimal integer 
     float_in_int = IEEE_754_Float_Raw_To_Int(src_float_ptr->ieee_754_format, 9, 31); 
  
    // Assign integer representation of mantissa 
    result_float.raw_ieee_754_format.raw.mantissa = float_in_int; 
  
    // Convert the exponent part (8 bits) to a corresponding decimal integer 
    float_in_int = IEEE_754_Float_Raw_To_Int(src_float_ptr->ieee_754_format, 1, 8); 
  
    // Assign integer representation of the exponent 
    result_float.raw_ieee_754_format.raw.exponient = float_in_int; 
  
    // Assign sign bit 
    result_float.raw_ieee_754_format.raw.sign = ((src_float_ptr->ieee_754_format >> 31) & 0x01 ); 
    memcpy(result_float_num_ptr, &result_float.raw_ieee_754_format.float_num, sizeof(float));
	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : IEEE_754_Float_Data_Status

DESCRIPTION    :   
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 15.04 

Bugs           :  
-*------------------------------------------------------------*/
uint16_t IEEE_754_Float_Data_Status(const float_ieee_754_format_t float_ieee_754_format, uint8_t *const float_data_status_ptr) 
{	
     if(float_data_status_ptr == NULL_DATA_PTR)
	 {
		 system_status_flag =  ERR_NULL_PTR;
		 Error_or_Warning_Proc("15.07.01", ERROR_OCCURED, system_status_flag); 
         return system_status_flag;		 
	 }
	 if(float_ieee_754_format.raw_ieee_754_format.raw.exponient == 255)		 
	 {
		 if(float_ieee_754_format.raw_ieee_754_format.raw.mantissa == 0)
		 {
		    if(float_ieee_754_format.raw_ieee_754_format.raw.sign == 1)
		    {
			   *float_data_status_ptr = REAL_VAL_IS_NEGATIVE_INFINITY;
			   return SUCCESS;
		    }
		    *float_data_status_ptr = REAL_VAL_IS_POSITIVE_INFINITY;
		    return SUCCESS;
		 }
		 else
		 {
			if(float_ieee_754_format.raw_ieee_754_format.raw.sign == 1)
		    {
			   *float_data_status_ptr = REAL_VAL_IS_NEGATIVE_NOT_A_NUM;
			   return SUCCESS;
		    }
		    *float_data_status_ptr = REAL_VAL_IS_POSITIVE_NOT_A_NUM;
		    return SUCCESS;
		 }
	 }
	 if(float_ieee_754_format.raw_ieee_754_format.raw.exponient == 0)  
	 {
		 if(float_ieee_754_format.raw_ieee_754_format.raw.mantissa == 0)
		 {
		     if(float_ieee_754_format.raw_ieee_754_format.raw.sign == 1)
		     {
			   *float_data_status_ptr = REAL_VAL_IS_NEGATIVE_ZERO;
			   return SUCCESS;
		     }
		     *float_data_status_ptr = REAL_VAL_IS_POSITIVE_ZERO;
		     return SUCCESS;
		 }
		 else
		 {
			 if(float_ieee_754_format.raw_ieee_754_format.raw.sign == 1)
		     {
			   *float_data_status_ptr = REAL_VAL_IS_NEGATIVE_DENORMALISED;
			   return SUCCESS;
		     }
		     *float_data_status_ptr = REAL_VAL_IS_POSITIVE_DENORMALISED;
		     return SUCCESS;
		 }
	 }
	 if(float_ieee_754_format.raw_ieee_754_format.raw.sign == 1)
	 {
		 *float_data_status_ptr = REAL_VAL_IS_NEGATIVE_NORMALISED;
		 return SUCCESS;
	 }
	 *float_data_status_ptr = REAL_VAL_IS_POSITIVE_NORMALISED;
	 return SUCCESS;
} 

/*------------------------------------------------------------*
FUNCTION NAME  : Int_To_Str

DESCRIPTION    : Converts a given integer num to string str_ptr. 
                 req_num_digits is the number of digits required in output. 
				 If req_num_digits is more than the number of digits in num, 
				 then 0s are added at the beginning. 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 15.05 

Bugs           :  
-*------------------------------------------------------------*/
int16_t Int_To_Str(const int32_t num, char *const str_ptr, const uint8_t req_num_digits) 
{ 
    uint32_t cur_num; 
    uint8_t str_len = 0; 
	
	if(str_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.05.01", ERROR_OCCURED, system_status_flag); 
		return EOF; 
	}
	if(num < 0)
	{
		cur_num = -num;		
	}
	else
	{
		cur_num = num;
	}
	if(req_num_digits <= MAX_FLOAT_PREC_SPECIFIER)
	{
		  while(cur_num) 
         { 
            str_ptr[str_len++] = (cur_num % BASE_10) + NUM_0_CHAR; 
            cur_num = cur_num / BASE_10; 
         }
		 // If number of digits required is more, then add 0s at the beginning 
         while (str_len < req_num_digits)
	     {		
            str_ptr[str_len++] = NUM_0_CHAR; 
	     }
	     if(num < 0)
         {
          	str_ptr[str_len++] = '-';
         }	
         if((Reverse_Str_Use_Same_Str(str_ptr, str_len)) != SUCCESS)
	     {
		    system_status_flag =  ERR_STR_REVERSE_PROC;
		    Error_or_Warning_Proc("15.05.02", ERROR_OCCURED, system_status_flag); 
		    return EOF;
	     }
	}
    else
	{
		 str_ptr[str_len++] = (cur_num % BASE_10) + NUM_0_CHAR; 
	}		
    str_ptr[str_len] = NULL_CHAR; 
    return str_len; 
}
 
/*------------------------------------------------------------*
FUNCTION NAME  : Real_Num_To_Str_Conv

DESCRIPTION    :  Converts a floating point number to string. 
								
INPUT          : 

OUTPUT         : 

NOTE           : Real_Num_To_Str_Conv() function is similar to ftoa()

Func ID        : 15.06 

Bugs           : 1: Most times, tested for float type, converted str's frac part = last digit of frac part in real num segment - 1.  
                    eg: if para_real num.float_num = 23.7843, then real_num_to_str_ptr = 23.784299. 
                    and if para_real num.float_num = 23.57, then real_num_to_str_ptr = 23.569999. 
					 
-*------------------------------------------------------------*/
uint16_t Real_Num_To_Str_Conv(char *const real_num_to_str_ptr, const real_num_t para_real_num, const uint8_t real_num_type, uint8_t num_digits_after_point)  
{ 
      // Extract frac part 
     real_num_t frac_part, extract_real_num_power_1; 
	
    // Extract integer part 
     int32_t int_part; 
   
    int16_t int_or_frac_part_str_len;    
	uint8_t i;
	
  	if(real_num_to_str_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.06.01", ERROR_OCCURED, system_status_flag); 
		return system_status_flag;
	}
	switch(real_num_type)
	{
		case FLOAT_FIXED_POINT_TYPE:
		   int_part = (int32_t) para_real_num.float_num;
		   frac_part.float_num = para_real_num.float_num - (float)int_part;
		break;
		case DOUBLE_FIXED_POINT_TYPE:
		   int_part = (int32_t) para_real_num.double_num;
		   frac_part.double_num = para_real_num.double_num - (double) int_part;
		break;
		case LONG_DOUBLE_FIXED_POINT_TYPE:
		   int_part = (int32_t) para_real_num.long_double_num;
		   frac_part.long_double_num = para_real_num.long_double_num - (long double) int_part;
		break;
		default:
		   system_status_flag =  ERR_FORMAT_INVALID;
		   Error_or_Warning_Proc("15.06.02", ERROR_OCCURED, system_status_flag); 
		   return system_status_flag;
	}
	memset(real_num_to_str_ptr, NULL_CHAR, 1);
	if((int_or_frac_part_str_len = Int_To_Str(int_part, real_num_to_str_ptr, 0)) == EOF)
	{	
        memset(real_num_to_str_ptr, NULL_CHAR, 1);    
		system_status_flag =  ERR_DATA_CONV_PROC;
		Error_or_Warning_Proc("15.06.03", ERROR_OCCURED, system_status_flag); 
		return system_status_flag;
	}
	if(num_digits_after_point > MAX_FLOAT_PREC_SPECIFIER)
	{
		num_digits_after_point = MAX_FLOAT_PREC_SPECIFIER;
	}
	// check for display option after point 
    if(num_digits_after_point != 0) 
    { 
        real_num_to_str_ptr[int_or_frac_part_str_len] = '.';  // add dot  
        // Get the value of fraction part upto given no. of points after dot.
        // The third parameter is needed to handle cases like 233.007 
		switch(real_num_type)
	    {
	    	case FLOAT_FIXED_POINT_TYPE:
		       if(frac_part.float_num < 0 )
		       {
			      frac_part.float_num = -frac_part.float_num; 
		       }
		    break;
		    case DOUBLE_FIXED_POINT_TYPE:
		      if(frac_part.double_num < 0 )
		      {
			      frac_part.double_num = -frac_part.double_num; 
		      }
		    break;
		    case LONG_DOUBLE_FIXED_POINT_TYPE:
		      if(frac_part.long_double_num < 0 )
		      {
			      frac_part.long_double_num = -frac_part.long_double_num; 
		      }
	    	break;
	    }
		for(i = 1; i <=num_digits_after_point; ++i)
		{
			switch(real_num_type)
	        {
	        	case FLOAT_FIXED_POINT_TYPE:
				   extract_real_num_power_1.float_num = frac_part.float_num * 10.0f;
				   int_part = (int32_t)extract_real_num_power_1.float_num;
				   #ifdef TRACE_DATA
				        printf("TRA: i = %d, float_num = %f, ", i , frac_part.float_num);
                   #endif
		        break;
		        case DOUBLE_FIXED_POINT_TYPE:
		           extract_real_num_power_1.double_num = frac_part.double_num * 10.0;
				   int_part = (int32_t)extract_real_num_power_1.double_num;
		        break;
		        case LONG_DOUBLE_FIXED_POINT_TYPE:
		           extract_real_num_power_1.long_double_num = frac_part.long_double_num * 10.0L;
				   int_part = (int32_t)extract_real_num_power_1.long_double_num;
	    	    break;
	        }			
			++int_or_frac_part_str_len;
			if((Int_To_Str(int_part, real_num_to_str_ptr + int_or_frac_part_str_len, MAX_FLOAT_PREC_SPECIFIER + 1)) == EOF)
		    {
		    	memset(real_num_to_str_ptr, NULL_CHAR, 1);
			    system_status_flag =  ERR_DATA_CONV_PROC;
		        Error_or_Warning_Proc("15.06.04", ERROR_OCCURED, system_status_flag); 
		        return system_status_flag;
		    }
			switch(real_num_type)
	        {
	        	case FLOAT_FIXED_POINT_TYPE:
				   frac_part.float_num = extract_real_num_power_1.float_num - (float)int_part;	
                   #ifdef TRACE_DATA
				        printf("next_float = %f\n", frac_part.float_num);
                   #endif				   
		        break;
		        case DOUBLE_FIXED_POINT_TYPE:
                   frac_part.double_num = extract_real_num_power_1.double_num - (double)int_part;				
		        break;
		        case LONG_DOUBLE_FIXED_POINT_TYPE:	
                   frac_part.long_double_num = extract_real_num_power_1.long_double_num - (long double)int_part;				
	    	    break;
	        }	
		}	
		real_num_to_str_ptr[int_or_frac_part_str_len + 1] = NULL_CHAR;	
    } 
    return SUCCESS;
} 

/*------------------------------------------------------------*
FUNCTION NAME  : Str_To_Real_Num_Conv

DESCRIPTION    :  Converts a string to floating point number. 
								
INPUT          : 

OUTPUT         : 

NOTE           : Str_To_Real_Num_Conv() function is similar to atof().
                 float in str is in base 10 and converted float is in base 10.  

Func ID        : 15.07 

Bugs           : 1: At times, frac part of converted real num tested for float type may not be exact value in as str.
                   eg for float type, if real_num_in_str_ptr = 39.876 then real_num_ptr->float_num  may not be 39.876000
				   
-*------------------------------------------------------------*/
uint16_t Str_To_Real_Num_Conv(real_num_t *const real_num_ptr, const char *const real_num_in_str_ptr, const uint8_t real_val_type, uint8_t num_digits_after_point) 
{
	 const char *real_num_str_cur_pos_ptr = real_num_in_str_ptr, *real_num_str_part_cur_pos_ptr;
     int16_t sign = 1, i;
     uint8_t inFraction = STATE_NO, num_digits_int_or_frac_part = 0; 	 
	 
	 if(real_num_str_cur_pos_ptr == NULL_DATA_PTR || real_num_ptr == NULL_DATA_PTR)
	 {
		 system_status_flag =  ERR_NULL_PTR;
		 Error_or_Warning_Proc("15.07.01", ERROR_OCCURED, system_status_flag); 
         return system_status_flag;
	 }
     switch(real_val_type)
	 {
		 case FLOAT_FIXED_POINT_TYPE:
		     real_num_ptr->float_num = 0.000000f; 
		 break;
		 case DOUBLE_FIXED_POINT_TYPE:
		    real_num_ptr->double_num = 0.000000000000000; 
		 break;
		 case LONG_DOUBLE_FIXED_POINT_TYPE:
		     real_num_ptr->long_double_num = 0.00000000000000000L; 
		 break;
		 default:
		    system_status_flag =  ERR_FORMAT_INVALID;
		    Error_or_Warning_Proc("15.07.02", ERROR_OCCURED, system_status_flag); 
            return system_status_flag;
	 }		 
     /* Take care of +/- sign */
     if (*real_num_str_cur_pos_ptr == '-')
     {
         ++real_num_str_cur_pos_ptr;
         sign = -1;
     }
     else if (*real_num_str_cur_pos_ptr == '+')
     {
         ++real_num_str_cur_pos_ptr;
     }	
	 if(num_digits_after_point > MAX_FLOAT_PREC_SPECIFIER)
	 {
		 num_digits_after_point = MAX_FLOAT_PREC_SPECIFIER;
	 }
     while(*real_num_str_cur_pos_ptr == '0')
	 {
		 ++real_num_str_cur_pos_ptr;
	 }
     if(*real_num_str_cur_pos_ptr == NULL_CHAR)
	 {
		 return SUCCESS;
	 }	
	 for(num_digits_int_or_frac_part = 0, real_num_str_part_cur_pos_ptr = real_num_str_cur_pos_ptr; *real_num_str_part_cur_pos_ptr != '.' && *real_num_str_part_cur_pos_ptr != NULL_CHAR; ++real_num_str_part_cur_pos_ptr, ++num_digits_int_or_frac_part)
	 {
		 if(*real_num_str_part_cur_pos_ptr < NUM_0_CHAR || *real_num_str_cur_pos_ptr > NUM_9_CHAR)
		 {
			  system_status_flag =  ERR_DATA_OUT_OF_RANGE;
		      Error_or_Warning_Proc("15.07.03", ERROR_OCCURED, system_status_flag); 
              return system_status_flag;
		 }
	 }
	 for(i = num_digits_int_or_frac_part - 1; i >= 0; --i, ++real_num_str_cur_pos_ptr)
	 {		
         switch(real_val_type)
	     {
	     	  case FLOAT_FIXED_POINT_TYPE:
		          real_num_ptr->float_num += ((float)(*real_num_str_cur_pos_ptr - NUM_0_CHAR) * Power_Of(BASE_10, i));
		      break;
		      case DOUBLE_FIXED_POINT_TYPE:
		          real_num_ptr->double_num += ((double)(*real_num_str_cur_pos_ptr - NUM_0_CHAR) * Power_Of(BASE_10, i)); 
		      break;
		      case LONG_DOUBLE_FIXED_POINT_TYPE:
		          real_num_ptr->long_double_num += ((long double)(*real_num_str_cur_pos_ptr - NUM_0_CHAR) * Power_Of(BASE_10, i));
		      break;
	     }		 		 
	 }
	 if(*real_num_str_part_cur_pos_ptr == NULL_CHAR)
	 {
		 return SUCCESS;
	 }
     if(*real_num_str_cur_pos_ptr == '.')
	 {
		 ++real_num_str_cur_pos_ptr;
	 }
	 for(num_digits_int_or_frac_part = 0, real_num_str_part_cur_pos_ptr = real_num_str_cur_pos_ptr; *real_num_str_part_cur_pos_ptr != NULL_CHAR; ++real_num_str_part_cur_pos_ptr, ++num_digits_int_or_frac_part)
	 {
		 if(*real_num_str_part_cur_pos_ptr < NUM_0_CHAR || *real_num_str_cur_pos_ptr > NUM_9_CHAR)
		 {
			  switch(real_val_type)
	          {
	          	  case FLOAT_FIXED_POINT_TYPE:
		            real_num_ptr->float_num = 0.000000f; 
	              break;
	        	  case DOUBLE_FIXED_POINT_TYPE:
		             real_num_ptr->double_num = 0.000000000000000; 
		          break;
		          case LONG_DOUBLE_FIXED_POINT_TYPE:
		              real_num_ptr->long_double_num = 0.00000000000000000L; 
		          break;
	          }	
			  system_status_flag =  ERR_DATA_OUT_OF_RANGE;
		      Error_or_Warning_Proc("15.07.03", ERROR_OCCURED, system_status_flag); 
              return system_status_flag;
		 }
	 }
	 if(num_digits_int_or_frac_part > num_digits_after_point)
	 {
		 num_digits_int_or_frac_part = num_digits_after_point;
	 }
	 for(i = 1; i <= num_digits_int_or_frac_part; ++i, ++real_num_str_cur_pos_ptr)
	 {		
         switch(real_val_type)
	     {
	     	  case FLOAT_FIXED_POINT_TYPE:
		          real_num_ptr->float_num += ((*real_num_str_cur_pos_ptr - NUM_0_CHAR) * (float)Power_Of(BASE_10, -i));
		      break;
		      case DOUBLE_FIXED_POINT_TYPE:
		          real_num_ptr->double_num += ((*real_num_str_cur_pos_ptr - NUM_0_CHAR) * Power_Of(BASE_10, -i)); 
		      break;
		      case LONG_DOUBLE_FIXED_POINT_TYPE:
		          real_num_ptr->long_double_num += ((*real_num_str_cur_pos_ptr - NUM_0_CHAR) * (long double)Power_Of(BASE_10, -i));
		      break;
	     }		 
	 }
	 switch(real_val_type)
	 {
	  	  case FLOAT_FIXED_POINT_TYPE:
	         real_num_ptr->float_num *= (float)sign; 
	      break;
	      case DOUBLE_FIXED_POINT_TYPE:
	         real_num_ptr->double_num *= (double)sign; 
	      break;
	      case LONG_DOUBLE_FIXED_POINT_TYPE:
	         real_num_ptr->long_double_num  *= (long double)sign;
	      break;
	 }
     return SUCCESS;
} 


/*------------------------------------------------------------*
FUNCTION NAME  : File_Get_Char

DESCRIPTION    : get char from stream
								
INPUT          : 

OUTPUT         : 

NOTE           : File_Get_Char operation is similar to fgetc()

Func ID        : 15.08 

Bugs           :  
-*------------------------------------------------------------*/
int16_t File_Get_Char(file_t *const fp)
{
	char get_char_arr[2];
	
	if(fp == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.08.01", ERROR_OCCURED, system_status_flag); 
		return (EOF);
	}	
	if(Read_Oper(fp, get_char_arr, 1) == EOF)
	{
		system_status_flag = ERR_DEV_OPER_READ_FUNC ;
		Error_or_Warning_Proc("15.08.02", ERROR_OCCURED, system_status_flag); 
		return (EOF);
	}
	return ((int16_t)get_char_arr[0]);
}

/*------------------------------------------------------------*
FUNCTION NAME  : Read_Oper

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : Read operation is similar to read() in linux

Func ID        : 15.09 

Bugs           :  
-*------------------------------------------------------------*/
int16_t Read_Oper(file_t *const fp, char *const out_buf_ptr, const uint8_t max_num_chars_to_read)
{
	char *out_buf_pos_ptr = out_buf_ptr;
	uint8_t num_chars_read = 0, stream_buf_before_proc_pos;
	
	if(out_buf_ptr == NULL_DATA_PTR || fp == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.09.01", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
    if(fp->stream_io_type != STREAM_IO_INPUT && fp->stream_io_type != STREAM_IO_BOTH)
	{
		system_status_flag =  ERR_FORMAT_INVALID;
		Error_or_Warning_Proc("15.09.02", ERROR_OCCURED, system_status_flag);
		*out_buf_pos_ptr = NULL_CHAR;	
		return EOF;
	}
	if(fp->stream_buf_cur_pos + fp->num_chars_stream_buf >= MAX_STREAM_BUFFER_SIZE)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE ;
		Error_or_Warning_Proc("15.09.03", ERROR_OCCURED, system_status_flag); 
		*out_buf_pos_ptr = NULL_CHAR;	
	    return EOF;
    }
	stream_buf_before_proc_pos = fp->stream_buf_cur_pos;
	if(fp->file_desp == STDIN_FILE_DESP)
	{
		// stdin_keyboard
		#ifdef KEYBOARD_MOD_ENABLE
		   out_buf_pos_ptr =  Get_Str(out_buf_ptr);
		   if(out_buf_pos_ptr == NULL_DATA_PTR)
		   {
			  system_status_flag = ERR_DEV_OPER_READ_FUNC ;
		      Error_or_Warning_Proc("15.09.04", ERROR_OCCURED, system_status_flag); 			  
			  return EOF;
		  }
		  num_chars_read = out_buf_pos_ptr - out_buf_ptr;		 
		#else
           system_status_flag = ERR_MOD_NOT_ENABLED;
		   Error_or_Warning_Proc("15.09.05", ERROR_OCCURED, system_status_flag); 
		   *out_buf_pos_ptr = NULL_CHAR;	
		   return EOF;			
		#endif
	}
	else
	{
	   for(num_chars_read = 0; num_chars_read < max_num_chars_to_read; ++num_chars_read)
	   {
	      if(fp->stream_buf_cur_pos == fp->num_chars_stream_buf || fp->stream_buf[fp->stream_buf_cur_pos] == NULL_CHAR || fp->stream_buf[fp->stream_buf_cur_pos] == ENTER_CHAR)
		  {
		    	break;
		  }
		  *out_buf_pos_ptr++ = fp->stream_buf[fp->stream_buf_cur_pos++];		 
	   }	  
	}	
	*out_buf_pos_ptr = NULL_CHAR;	
    return num_chars_read;	
}

/*------------------------------------------------------------*
FUNCTION NAME  : Get_Char

DESCRIPTION    : Get_Char() operation is similar to getchar()
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 04.19 

Bugs           :  
-*------------------------------------------------------------*/
char Get_Char(void)
{
	char last_read_char, disp_char;
		
	last_read_char = getchar();
	if(last_read_char == NULL_CHAR)
	{
		return NULL_CHAR;
	}
	// committed to prevent explicit echo char
	/*
	if((disp_char = Put_Char(last_read_char)) == NULL_CHAR)
	{
		return NULL_CHAR;
	} */
	return last_read_char;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Get_Str

DESCRIPTION    : Get_Str() operation is similar to gets()
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 04.20 

Bugs           :  
-*------------------------------------------------------------*/
char *Get_Str(char *const get_str)
{
	char *read_ptr = get_str;
    char last_read_char = NULL_CHAR, disp_char;
	uint8_t num_chars_read_str = 0;	
	
	if(get_str == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
	    Error_or_Warning_Proc("04.20.01", ERROR_OCCURED, system_status_flag);
	    return NULL_DATA_PTR;
	}
	read_ptr = get_str;
    do
	{
		if(stdin_keyboard.num_chars_stream_buf + stdin_keyboard.stream_buf_cur_pos + num_chars_read_str + 1 > MAX_STREAM_BUFFER_SIZE )
		{
			system_status_flag = ERR_DATA_OUT_OF_RANGE;
	        Error_or_Warning_Proc("04.20.02", ERROR_OCCURED, system_status_flag);            
			*read_ptr = NULL_CHAR;
	        return NULL_DATA_PTR;
		}
        last_read_char = Stream_Stdin_Oper(0);
		if(last_read_char != NULL_CHAR && last_read_char != EOF )
		{		       
			if(last_read_char != BACKSPACE_CHAR && last_read_char != ENTER_CHAR)
			{
			   *read_ptr = last_read_char;
			   ++num_chars_read_str;
			   ++read_ptr;
               *read_ptr = NULL_CHAR;	   
		    }
		    else
		    {
			   if(last_read_char == BACKSPACE_CHAR && num_chars_read_str > 0 )
			   {
				   --num_chars_read_str;
				   --read_ptr;
				   *read_ptr = NULL_CHAR;				   
			   }
		    }
		}			
	}
    while(last_read_char != ENTER_CHAR && last_read_char != EOF);
	*read_ptr = NULL_CHAR;
	File_Flush(&stdin_keyboard);	
	return read_ptr; 	
}

/*------------------------------------------------------------*
FUNCTION NAME  : Stream_Stdin_Oper

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 04.21 

Bugs           :  
-*------------------------------------------------------------*/
static int16_t Stream_Stdin_Oper(const uint8_t keyboard_ch_id)
{
	file_t *stdin_ptr = &stdin_keyboard;
	uint8_t ret_status;
	char last_read_char = NULL_CHAR;
  
	last_read_char = Get_Char();
	if(last_read_char != BACKSPACE_CHAR)
	{
		 if(stdin_ptr->stream_buf_cur_pos + stdin_ptr->num_chars_stream_buf + 1 >= MAX_STREAM_BUFFER_SIZE)
		 {
		 	 system_status_flag = ERR_DATA_OUT_OF_RANGE;
	         Error_or_Warning_Proc("04.21.01", ERROR_OCCURED, system_status_flag);
	  		 return EOF;                     
	     }
		 stdin_ptr->stream_buf[stdin_ptr->stream_buf_cur_pos + stdin_ptr->num_chars_stream_buf] = last_read_char;
		 ++stdin_ptr->num_chars_stream_buf; 
		 stdin_ptr->stream_buf[stdin_ptr->stream_buf_cur_pos + stdin_ptr->num_chars_stream_buf] = NULL_CHAR;
    }				
    else
	{
	    //read char is BACKSPACE_CHAR
	    if(stdin_ptr->num_chars_stream_buf > 0)
		{
		  --stdin_ptr->num_chars_stream_buf; 
		  stdin_ptr->stream_buf[stdin_ptr->stream_buf_cur_pos + stdin_ptr->num_chars_stream_buf] = NULL_CHAR;			     							                     
		}
	}   
	return last_read_char;
}	
	
/*------------------------------------------------------------*
FUNCTION NAME  : Scan

DESCRIPTION    : Scan() operation is similar to scanf()
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 04.18 

Bugs           :  
-*------------------------------------------------------------*/
int16_t Scan(const char *const format_str,...) 
{ 
    va_list ap;
	int16_t count;
	
	if(format_str == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("04.18.01", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	if(*format_str == NULL_CHAR)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("04.18.02", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	va_start(ap, format_str);
    count = VFile_Scan(&stdin_keyboard, format_str, &ap);
	va_end(ap);
	if(count == EOF)
	{
		system_status_flag = ERR_EOF;
		Error_or_Warning_Proc("04.18.03", ERROR_OCCURED, system_status_flag);
	}
	return(count);
} 

/*------------------------------------------------------------*
FUNCTION NAME  : File_Get_Str

DESCRIPTION    : get string from stream
								
INPUT          : 

OUTPUT         : 

NOTE           : File_Get_Str() operation is similiar to fgets()

Func ID        : 15.10 

Bugs           :  
-*------------------------------------------------------------*/
char *File_Get_Str(char *const dst_str_ptr,  file_t *const fp, const uint8_t max_chars_to_get)
{
	char *char_ptr;
	int16_t num_chars_read;
	
	if(fp == NULL_DATA_PTR || dst_str_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.10.01", ERROR_OCCURED, system_status_flag); 
		return (NULL_DATA_PTR);
	}
	char_ptr = dst_str_ptr;
	if((num_chars_read = Read_Oper(fp, dst_str_ptr, max_chars_to_get)) == EOF)
	{
		system_status_flag = ERR_DEV_OPER_READ_FUNC ;
		Error_or_Warning_Proc("15.10.02", ERROR_OCCURED, system_status_flag); 
		return (NULL_DATA_PTR);
	}	
	char_ptr += num_chars_read;
	return (char_ptr);
}

/*------------------------------------------------------------*
FUNCTION NAME  : VFile_Scan

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : VFile_Scan() operation is similar to vfscanf()

Func ID        : 15.11 

Bugs           :  
-*------------------------------------------------------------*/
int16_t VFile_Scan(file_t *const fp, const char *const fmt, va_list *const scan_arg_list_ptr)
{
    char  in_buf[MAX_STREAM_BUFFER_SIZE + 1];
    int16_t count;
	
	if(scan_arg_list_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.11.01", ERROR_OCCURED, system_status_flag);
	    return (EOF);
	}
    if (File_Get_Str(in_buf, fp, MAX_STREAM_BUFFER_SIZE) == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_DEV_OPER_READ_FUNC;
		Error_or_Warning_Proc("15.11.02", ERROR_OCCURED, system_status_flag);
	    return (EOF);
	}	
    count = Scan_Data(in_buf, fmt, scan_arg_list_ptr);
	if(count == EOF)
	{
		system_status_flag =  ERR_DEV_OPER_READ_FUNC;
		Error_or_Warning_Proc("15.11.03", ERROR_OCCURED, system_status_flag);
	}
	return (count);
}

/*------------------------------------------------------------*
FUNCTION NAME  : File_Scan

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : File_Scan() operation is similar to fscanf()

Func ID        : 15.12 

Bugs           :  
-*------------------------------------------------------------*/
int16_t File_Scan(file_t *const fp, const char *const fmt, ...)
{
  va_list ap;
  int16_t    count;
	
	if(fmt == NULL_DATA_PTR || fp == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.12.01", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
	if(*fmt == NULL_CHAR)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("15.12.02", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
    va_start(ap, fmt);
	count = VFile_Scan(fp, fmt, &ap);
	va_end(ap);
	if(count == EOF)
	{
		system_status_flag =  ERR_DEV_OPER_READ_FUNC;
		Error_or_Warning_Proc("15.12.03", ERROR_OCCURED, system_status_flag); 
	}
	return(count);
}

/*------------------------------------------------------------*
FUNCTION NAME  : Str_Scan

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : Str_Scan() operation is similar to sscanf()

Func ID        : 15.13 

Bugs           :  
-*------------------------------------------------------------*/		    			    
int16_t Str_Scan(const char *const ibuf, const char *const fmt, ...)
{
	va_list ap;
	int16_t count;
	
	if(ibuf == NULL_DATA_PTR )
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.13.01", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
	if(fmt == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.13.02", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
	if(*ibuf == NULL_CHAR)
	{
		system_status_flag =  ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("15.13.03", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
	if(*fmt == NULL_CHAR)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("15.13.04", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	va_start(ap, fmt);
	count = Scan_Data(ibuf, fmt, &ap);
	va_end(ap);
	if(count == EOF)
	{
		system_status_flag =  ERR_DEV_OPER_READ_FUNC;
		Error_or_Warning_Proc("15.13.05", ERROR_OCCURED, system_status_flag); 
	}
	return(count);
}

/*------------------------------------------------------------*
FUNCTION NAME  : Scan_Data

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : Scan_Data() operation is similar to vsscanf()

Func ID        : 15.14 

Bugs           :  
-*------------------------------------------------------------*/
static int16_t Scan_Data(const char *inp, const char  *const fmt0, va_list *const scan_va_list_ptr)
{
	size_t sum = 0;
	int inr;
	const char *fmt = (const char *)fmt0;
	int c;			/* character from format, or conversion */
	size_t width;		/* field width, or 0 */
	char *p;		/* points into all kinds of strings */
	int n;			/* handy integer */
	int flags;		/* flags as defined above */
	char *p0;		/* saves original value of p when necessary */
	int nassigned;		/* number of fields assigned */
	int nconversions;	/* number of conversions */
	int nread;		/* number of characters consumed from fp */
	int base;		/* base argument to conversion function */
	char ccltab[256];	/* character class table for %[...] */
	char buf[MAX_STREAM_BUFFER_SIZE];		/* buffer for numeric conversions */     
	/* `basefix' is used to avoid `if' tests in the integer scanner */
	static short basefix[17] =
		{ 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    int8_t num_digits_after_point, num_digits_before_point, num_chars_in_buf = 0; 
	
	if(scan_va_list_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.14.01", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
	inr = strlen(inp);
	if(inr == 0)
	{
		system_status_flag =  ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("15.14.02", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
	nassigned = 0;
	nconversions = 0;
	nread = 0;
	base = 0;		/* XXX just to keep gcc happy */
	for (;;)
	{
		c = *fmt++;
		if (c == NULL_CHAR)
			return (nassigned);
		if (isspace(c))
		{
			while (inr > 0 && isspace(*inp))
			{
				nread++, inr--, inp++;
			}
			continue;
		}
		if (c != '%')
			goto literal;
		width = 0;
		flags = 0;
		/*
		 * switch on the format.  continue if done;
		 * break once format type is derived.
		 */
again:		c = *fmt++;
		switch (c)
		{
		case '%':
literal:
			if (inr <= 0)
				goto input_failure;
			if (*inp != c)
				goto match_failure;
			inr--, inp++;
			nread++;
			continue;

		case '*':
			flags |= SUPPRESS;
			goto again;
		case 'l':
			if (flags & LONG)
			{
				flags &= ~LONG;
				flags |= LONGLONG;
			}
			else
				flags |= LONG;
			goto again;
		case 'L':
        	flags |= LONGLONG;	
			goto again;	
		case 'q':
			flags |= LONGLONG;	/* not quite */
			goto again;
		case 'h':
			if (flags & SHORT)
			{
				flags &= ~SHORT;
				flags |= SHORTSHORT;
			}
			else
				flags |= SHORT;
			goto again;

		case '0': case '1': case '2': case '3': case '4':
		case '5': case '6': case '7': case '8': case '9':
			width = width * 10 + c - '0';
			goto again;

		/*
		 * Conversions.
		 */
		case 'd':
			c = CT_INT;
			base = BASE_10;
			break;

		case 'i':
			c = CT_INT;
			base = 0;
			break;

		case 'o':
			c = CT_INT;
			flags |= UNSIGNED;
			base = BASE_08;
			break;

		case 'u':
			c = CT_INT;
			flags |= UNSIGNED;
			base = BASE_10;
			break;

		case 'X':
		case 'x':
			flags |= PFXOK;	/* enable 0x prefixing */
			c = CT_INT;
			flags |= UNSIGNED;
			base = BASE_16;
			break;
		case 's':
			c = CT_STRING;
			break;
        case 'f':
		    c = CT_REAL;
			flags |= REAL;
			base = BASE_10;			
        break;		
		case '[':
			fmt = Scan_Input_Limit_Chars(ccltab, fmt);
			flags |= NOSKIP;
			c = CT_CCL;
			break;

		case 'c':
			flags |= NOSKIP;
			c = CT_CHAR;
			break;

		case 'p':	/* pointer format is like hex */
			flags |= POINTER | PFXOK;
			c = CT_INT;
			flags |= UNSIGNED;
			base = BASE_16;
			break;

		case 'n':
			nconversions++;
			if (flags & SUPPRESS)	/* ??? */
				continue;
			if (flags & SHORTSHORT)
				*va_arg(*scan_va_list_ptr, char *) = nread;
			else if (flags & SHORT)
				*va_arg(*scan_va_list_ptr, short *) = nread;
			else if (flags & LONG)
				*va_arg(*scan_va_list_ptr, long *) = nread;
			else if (flags & LONGLONG)
				*va_arg(*scan_va_list_ptr, long long *) = nread;
			else
				*va_arg(*scan_va_list_ptr, int *) = nread;
			continue;
		}

		/*
		 * We have a conversion that requires input.
		 */
		if (inr <= 0)
			goto input_failure;

		/*
		 * Consume leading white space, except for formats
		 * that suppress this.
		 */
		if ((flags & NOSKIP) == 0)
		{
			while (isspace(*inp))
			{
				nread++;
				if (--inr > 0)
					inp++;
				else 
					goto input_failure;
			}
			/*
			 * Note that there is at least one character in
			 * the buffer, so conversions that do not set NOSKIP
			 * can no longer result in an input failure.
			 */
		}

		/*
		 * Do the conversion.
		 */
		switch (c)
		{
		   case CT_CHAR:
			/* scan arbitrary characters (sets NOSKIP) */
			if (width == 0)
				width = 1;
			if (flags & SUPPRESS)
			{
      
				memset(va_arg(*scan_va_list_ptr, char *), NULL_CHAR , 1);								
				for (;;)
				{
					if ((n = inr) < (int)width)
					{
						sum += n;
						width -= n;
						inp += n;
						if (sum == 0)
							goto input_failure;
						break;
					} 
					else
					{
						sum += width;
						inr -= width;
						inp += width;
						break;
					}
				}
				nread += sum;
			} 
			else
			{
				memcpy(va_arg(*scan_va_list_ptr, char *), inp, width);
				inr -= width;
				inp += width;
				nread += width;
				nassigned++;
			}
			nconversions++;
			break;

		case CT_CCL:
			/* scan a (nonempty) character class (sets NOSKIP) */
	    	/*	
			   if (width == 0)
				width = (size_t)~0; // size is infinity 
			*/
			if (width == 0 || width > sizeof(buf) - 1)
				width = sizeof(buf) - 1;
			/* take only those things in the class */
			if (flags & SUPPRESS)
			{
				n = 0;	
				memset(va_arg(*scan_va_list_ptr, char *), NULL_CHAR, 1);			
				while (ccltab[(unsigned char)*inp])
				{
					n++, inr--, inp++;
					if (--width == 0)
					{
						break;
					}
					if (inr <= 0)
					{
						if (n == 0)
							goto input_failure;					    						   
						break;
					}
				}
				if (n == 0)
					goto match_failure;					
			}
			else
			{
				p0 = p = va_arg(*scan_va_list_ptr, char *);
				while (ccltab[(unsigned char)*inp])
				{
					inr--;
					*p++ = *inp++;
					if (--width == 0)
						break;
					if (inr <= 0)
					{
						if (p == p0)
							goto input_failure;
						break;
					}
				}
				n = p - p0;
				if (n == 0)
					goto match_failure;
				*p = 0;
				nassigned++;
			}
			nread += n;
			nconversions++;
			break;
		case CT_STRING:
			/* like CCL, but zero-length string OK, & no NOSKIP */
	 	/*	
		    if (width == 0)
				width = (size_t)~0; // size is infinity
		*/
		   	if (width == 0 || width > sizeof(buf) - 1)
				width = sizeof(buf) - 1;	
			if (flags & SUPPRESS)
			{
				n = 0;
				memset(va_arg(*scan_va_list_ptr, char *), NULL_CHAR, 1);
				while (!isspace(*inp))
				{
					n++, inr--, inp++;
					if (--width == 0)
						break;
					if (inr <= 0)
						break;
				}
				nread += n;
			}
			else
			{
				p0 = p = va_arg(*scan_va_list_ptr, char *);
				while (!isspace(*inp))
				{
					inr--;
					*p++ = *inp++;
					if (--width == 0)
						break;
					if (inr <= 0)
						break;
				}
				*p = NULL_CHAR;
				nread += p - p0;
				nassigned++;
			}
			nconversions++;
			continue;
			
		case CT_INT:
			/* scan an integer as if by the conversion function */
/*#ifdef hardway
			if (width == 0 || width > sizeof(buf) - 1)
				width = sizeof(buf) - 1;
#else
			// size_t is unsigned, hence this optimisation 
			if (--width > sizeof(buf) - 2)
				width = sizeof(buf) - 2;
			width++;
#endif */
           if (width == 0 || width > MAX_WIDTH_SPECIFIER)
				width = MAX_WIDTH_SPECIFIER;
			memset(buf, NULL_CHAR, MAX_STREAM_BUFFER_SIZE);
           	flags |= SIGNOK | NDIGITS | NZDIGITS;
			for (p = buf; width; width--)
			{
				c = *inp;
				/*
				 * Switch on the character; `goto int_ok'
				 * if we accept it as a part of number.
				 */
				switch (c)
				{

				/*
				 * The digit 0 is always legal, but is
				 * special.  For %i conversions, if no
				 * digits (zero or nonzero) have been
				 * scanned (only signs), we will have
				 * base==0.  In that case, we should set
				 * it to 8 and enable 0x prefixing.
				 * Also, if we have not scanned zero digits
				 * before this, do not turn off prefixing
				 * (someone else will turn it off if we
				 * have scanned any nonzero digits).
				 */
				case '0':
					if (base == 0)
					{
						base = BASE_08;
						flags |= PFXOK;
					}
					if (flags & NZDIGITS)
					    flags &= ~(SIGNOK|NZDIGITS|NDIGITS);
					else
					    flags &= ~(SIGNOK|PFXOK|NDIGITS);
					goto int_ok;

				/* 1 through 7 always legal */
				case '1': case '2': case '3':
				case '4': case '5': case '6': case '7':
					base = basefix[base];
					flags &= ~(SIGNOK | PFXOK | NDIGITS);
					goto int_ok;

				/* digits 8 and 9 ok if decimal or hex */
				case '8': case '9':
					base = basefix[base];
					if (base <= BASE_08)
						break;	/* not legal here */
					flags &= ~(SIGNOK | PFXOK | NDIGITS);
					goto int_ok;

				/* letters ok if hex */
				case 'A': case 'B': case 'C':
				case 'D': case 'E': case 'F':
				case 'a': case 'b': case 'c':
				case 'd': case 'e': case 'f':
					/* no need to fix base here */
					if (base <= BASE_10)
						break;	/* not legal here */
					flags &= ~(SIGNOK | PFXOK | NDIGITS);
					goto int_ok;

				/* sign ok only as first character */
				case '+': case '-':
					if (flags & SIGNOK)
					 {
						flags &= ~SIGNOK;
						goto int_ok;
					}
					break;

				/* x ok if flag still set & 2nd char */
				case 'x': case 'X':
					if (flags & PFXOK && p == buf + 1)
					 {
						base = BASE_16;	/* if %i */
						flags &= ~PFXOK;
						goto int_ok;
					}
					break;
				}

				/*
				 * If we got here, c is not a legal character
				 * for a number.  Stop accumulating digits.
				 */
				break;
		int_ok:
				/*
				 * c is legal: store it and look at the next.
				 */
				*p++ = c;
				if (--inr > 0)
					inp++;
				else 
					break;		/* end of input */
			}
			/*
			 * If we had only a sign, it is no good; push
			 * back the sign.  If the number ends in `x',
			 * it was [sign] '0' 'x', so push back the x
			 * and treat it as [sign] '0'.
			 */
			if (flags & NDIGITS)
			 {
				if (p > buf)
				{
					inp--;
					inr++;
				}
				goto match_failure;
			}
			c = ((char *)p)[-1];
			if (c == 'x' || c == 'X')
			 {
				--p;
				inp--;
				inr++;
			}
			if (flags & SUPPRESS)
			{
				if (flags & POINTER)
					*va_arg(*scan_va_list_ptr, void **) = NULL_DATA_PTR;
				else if (flags & SHORTSHORT)
					*va_arg(*scan_va_list_ptr, char *) = NULL_CHAR;
				else if (flags & SHORT)
					*va_arg(*scan_va_list_ptr, short *) = 0;
				else if (flags & LONG)
					*va_arg(*scan_va_list_ptr, long *) = 0;
				else if (flags & LONGLONG)
					*va_arg(*scan_va_list_ptr, long long *) = 0;
				else
					*va_arg(*scan_va_list_ptr, int *) = 0;
				
			}
			else
			{
			    u_quad_t res;

				*p = NULL_CHAR;
				if (flags & UNSIGNED) 
					 res = strtoul(buf, (char **)NULL_DATA_PTR, base);				   
				else
				    res = strtoq(buf, (char **)NULL_DATA_PTR, base);
				if (flags & POINTER)
					*va_arg(*scan_va_list_ptr, void **) = (void *)(ptr_t)res;
				else if (flags & SHORTSHORT)
					*va_arg(*scan_va_list_ptr, char *) = res;
				else if (flags & SHORT)
					*va_arg(*scan_va_list_ptr, short *) = res;
				else if (flags & LONG)
					*va_arg(*scan_va_list_ptr, long *) = res;
				else if (flags & LONGLONG)
					*va_arg(*scan_va_list_ptr, long long *) = res;
				else
					*va_arg(*scan_va_list_ptr, int *) = res;
				nassigned++; 	
			}
			nread += p - buf;
			nconversions++;
		break;
        case CT_REAL:
		     if (width == 0 || width > MAX_WIDTH_SPECIFIER + 1 + MAX_FLOAT_PREC_SPECIFIER)				 
				width = MAX_WIDTH_SPECIFIER + 1 + MAX_FLOAT_PREC_SPECIFIER;
			num_digits_after_point = 0;
			num_digits_before_point = 0;
			num_chars_in_buf = 0;
			memset(buf, NULL_CHAR, MAX_STREAM_BUFFER_SIZE);
			flags |= SIGNOK | DPTOK |NDIGITS| NZDIGITS;
			for (p = buf; width; width--)
			{
				c = *inp;
				switch(c)
				{
					case '0': case '1':	case '2': case '3':
				    case '4': case '5': case '6': case '7':	
                    case '8': case '9':	
					  if(flags & NZDIGITS)
					  {
					      flags &= ~(SIGNOK | NDIGITS | NZDIGITS);						  
					  }
					  if(flags & DPTOK)
					  {
					  	 if(++num_digits_before_point > MAX_WIDTH_SPECIFIER)
						 {
							  goto match_failure;  							  
						 }
                         else
						 {
							  buf[num_chars_in_buf++] = c; 
						 }						  
					  }
					  else
					  {
						  if(++num_digits_after_point > MAX_FLOAT_PREC_SPECIFIER)
						  {
							  buf[num_chars_in_buf] = NULL_CHAR;
							  //discard extra numeric data if precison exceeds limit
						  }
                          else
						  {
							 buf[num_chars_in_buf++] = c; 
						  }	
					  }
					  goto real_ok;
				   //break;
				   case '+': 
				   case '-':
					 if (flags & SIGNOK )
					 {
						flags &= ~(SIGNOK);
						if(c == '-')
						{
					    	buf[num_chars_in_buf++] = '-';							
						}
						goto real_ok;
					 }
				   break;
				   case '.':
				     if (flags & DPTOK)
					 {
						flags &= ~(DPTOK);
						buf[num_chars_in_buf++] = '.';
						goto real_ok;
					 }
				   break; 
                   			
				}
				/*
				 * If we got here, c is not a legal character
				 * for a number.  Stop accumulating digits.
				 */
				break;
		real_ok:
				/*
				 * c is legal: store it and look at the next.
				 */
				*p++ = c;
				if (--inr > 0)
					inp++;
				else 
					break;		/* end of input */
			}	
			//If we had only a sign, it is no good;
			if (flags & NDIGITS)
			{
			     goto match_failure;
			}
			if(flags & SUPPRESS)
			{
				if (flags & LONG)
					*va_arg(*scan_va_list_ptr, double *) = (double)0.0;
				else if (flags & LONGLONG)
					*va_arg(*scan_va_list_ptr, long double *) = (long double)0.0;
				else
					*va_arg(*scan_va_list_ptr, float *) = (float)0.0;				
			}
			else
			{
				real_num_t real_num;
							
			    *p = NULL_CHAR;
				if (flags & LONG)
				{
					if((Str_To_Real_Num_Conv(&real_num, buf, DOUBLE_FIXED_POINT_TYPE, num_digits_after_point)) != SUCCESS)
				    {
				     	goto input_failure;
				    }
					*va_arg(*scan_va_list_ptr, double *) = real_num.double_num;
				}
				else if (flags & LONGLONG)
				{
					if((Str_To_Real_Num_Conv(&real_num, buf, LONG_DOUBLE_FIXED_POINT_TYPE, num_digits_after_point)) != SUCCESS)
				    {
				     	goto input_failure;
				    }
					*va_arg(*scan_va_list_ptr, long double *) = real_num.long_double_num; 
				}
				else
				{
					if((Str_To_Real_Num_Conv(&real_num, buf, FLOAT_FIXED_POINT_TYPE, num_digits_after_point)) != SUCCESS)
				    {
				     	goto input_failure;
				    }
					*va_arg(*scan_va_list_ptr, float *) = real_num.float_num;
				}
				nassigned++;
			}
			nread += p - buf;
			nconversions++;
		break;
		}
	}
input_failure:
	return (nconversions != 0 ? nassigned : EOF);
match_failure:
	return (nassigned);
}

/*------------------------------------------------------------*
FUNCTION NAME  : Scan_Input_Limit_Chars

DESCRIPTION    : Fill in the given table from the scanset at the given format
                 (just after `[').  Return a pointer to the character past the
                  closing `]'.  The table has a 1 wherever characters should be
                 considered part of the scanset.
								
INPUT          : 

OUTPUT         : 

NOTE           : Scan_Input_Limit_Chars operation is similar to __sccl()

Func ID        : 15.15 

Bugs           :  
-*------------------------------------------------------------*/
 static const char *Scan_Input_Limit_Chars(char *tab, const char *fmt)
{
	int c, n, v;

	/* first `clear' the whole table */
	c = *fmt++;		/* first char hat => negated scanset */
	if (c == '^')
    {
		v = 1;		/* default => accept */
		c = *fmt++;	/* get new first char */
	} 
	else
	{
		v = 0;		/* default => reject */
	}

	/* XXX: Will not work if sizeof(tab*) > sizeof(char) */
	(void) memset(tab, v, 256);

	if (c == 0)
	{
		return (fmt - 1);/* format ended before closing ] */
	}

	/*
	 * Now set the entries corresponding to the actual scanset
	 * to the opposite of the above.
	 *
	 * The first character may be ']' (or '-') without being special;
	 * the last character may be '-'.
	 */
	v = 1 - v;
	for (;;)
	{
		tab[c] = v;		/* take character c */
doswitch:
		n = *fmt++;		/* and examine the next */
		switch (n)
		{

		case 0:			/* format ended too soon */
			return (fmt - 1);

		case '-':
			/*
			 * A scanset of the form
			 *	[01+-]
			 * is defined as `the digit 0, the digit 1,
			 * the character +, the character -', but
			 * the effect of a scanset such as
			 *	[a-zA-Z0-9]
			 * is implementation defined.  The V7 Unix
			 * scanf treats `a-z' as `the letters a through
			 * z', but treats `a-a' as `the letter a, the
			 * character -, and the letter a'.
			 *
			 * For compatibility, the `-' is not considerd
			 * to define a range if the character following
			 * it is either a close bracket (required by ANSI)
			 * or is not numerically greater than the character
			 * we just stored in the table (c).
			 */
			n = *fmt;
			if (n == ']' || n < c)
			{
				c = '-';
				break;	/* resume the for(;;) */
			}
			fmt++;
			/* fill in the range */
			do
			{
			    tab[++c] = v;
			} while (c < n);
			c = n;
			/*
			 * Alas, the V7 Unix scanf also treats formats
			 * such as [a-c-e] as `the letters a through e'.
			 * This too is permitted by the standard....
			 */
			goto doswitch;
//		break;

		case ']':		/* end of scanset */
			return (fmt);

		default:		/* just another character */
			c = n;
			break;
		}
	}
	/* NOT REACHED */
	return NULL_DATA_PTR;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Print

DESCRIPTION    : Print() operation is similar to printf()
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 10.18 

Bugs           :  
-*------------------------------------------------------------*/
int16_t Print(const char* const format_ptr,...) 
{ 
   va_list arg;
   int16_t num_chars_printed = 0;	
	
	if(format_ptr == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("10.18.01", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	if(*format_ptr == NULL_CHAR)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("10.18.02", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	va_start(arg, format_ptr); 
	num_chars_printed = VFile_Print(&stdout_vt, format_ptr, &arg);
	va_end(arg);
	if(num_chars_printed == EOF)
	{
		system_status_flag = ERR_EOF;
		Error_or_Warning_Proc("10.18.03", ERROR_OCCURED, system_status_flag);
	}
	return num_chars_printed;
  
} 

/*------------------------------------------------------------*
FUNCTION NAME  : Put_Char

DESCRIPTION    : Put_Char() operation is similar to putchar()
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 10.19 

Bugs           :  
-*------------------------------------------------------------*/
char Put_Char(const char to_disp_char)
{
	file_t *stdout_ptr = &stdout_vt;
	char displayed_char;
	uint8_t ret_status;
	
	displayed_char = putchar(to_disp_char);
	if(displayed_char == NULL_CHAR)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
	    Error_or_Warning_Proc("10.19.01", ERROR_OCCURED, system_status_flag );
	    return NULL_CHAR;
	}
	if(stdout_ptr->num_chars_stream_buf + stdout_ptr->stream_buf_cur_pos + 1 >= MAX_STREAM_BUFFER_SIZE )
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
	    Error_or_Warning_Proc("10.19.02", ERROR_OCCURED, system_status_flag);
	    return NULL_CHAR;
	}
	return displayed_char;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Put_Str

DESCRIPTION    : Put_Str() operation is similar to puts()
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 10.20 

Bugs           :  
-*------------------------------------------------------------*/
int16_t Put_Str(const char *const to_disp_str)
{
	file_t *stdout_ptr = &stdout_vt;
	uint8_t num_chars_displayed = 0;
	uint8_t ret_status;
	
	if(to_disp_str == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
	    Error_or_Warning_Proc("10.20.01", ERROR_OCCURED, system_status_flag);
	    return EOF;
	}
	if((ret_status = strlen(to_disp_str)) == 0)
	{
		system_status_flag = ERR_STR_LEN_OPER;
	    Error_or_Warning_Proc("10.20.02", ERROR_OCCURED, system_status_flag);
	    return EOF;
	}
	if(stdout_ptr->num_chars_stream_buf + stdout_ptr->stream_buf_cur_pos + 1 >=  MAX_STREAM_BUFFER_SIZE )
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
	    Error_or_Warning_Proc("10.20.03", ERROR_OCCURED, system_status_flag);
	    return EOF;	     
	}
	puts(to_disp_str);
	putchar(NEW_LINE_CHAR);	
	return num_chars_displayed;		
}

/*------------------------------------------------------------*
FUNCTION NAME  : File_Put_Char

DESCRIPTION    : put char into stream
								
INPUT          : 

OUTPUT         : 

NOTE           : File_Put_Char() operation is similar to fputc()

Func ID        : 15.16 

Bugs           :  
-*------------------------------------------------------------*/
int16_t File_Put_Char(const char put_char, file_t *const fp)
{
	int16_t num_chars_written;
	char put_char_arr[2];
	
	if(fp == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.16.01", ERROR_OCCURED, system_status_flag); 
		return (EOF);
	}
	put_char_arr[0] = put_char;
	put_char_arr[1] = NULL_CHAR;
	if((num_chars_written = Write_Oper(fp, put_char_arr, 1)) == EOF)
	{
		system_status_flag = ERR_DEV_OPER_WRITE_FUNC ;
		Error_or_Warning_Proc("15.16.02", ERROR_OCCURED, system_status_flag); 
		return (EOF);
	}
	if(num_chars_written == 0)
	{
		return 0;
	}
	return ((int16_t )put_char);
}

/*------------------------------------------------------------*
FUNCTION NAME  : Write_Oper

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : Write_Oper() operation is similar to write() in linux

Func ID        : 15.17

Bugs           :  
-*------------------------------------------------------------*/
int16_t Write_Oper(file_t *const fp, const char *const in_buf_ptr, const uint8_t max_num_chars_to_write)
{
	const char *in_buf_pos_ptr = in_buf_ptr;
	int16_t num_chars_written = 0, stream_buf_before_proc_pos;
	
	if(in_buf_ptr == NULL_DATA_PTR || fp == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.17.01", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
    if(fp->stream_io_type != STREAM_IO_OUTPUT && fp->stream_io_type != STREAM_IO_BOTH)
	{
		system_status_flag =  ERR_FORMAT_INVALID;
		Error_or_Warning_Proc("15.17.02", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
	stream_buf_before_proc_pos = fp->stream_buf_cur_pos;
	if(*in_buf_pos_ptr == NULL_CHAR)
	{
		system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		Error_or_Warning_Proc("15.17.03", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
	if(fp->num_chars_stream_buf == 0)
	{
		return 0;
    }
	if(fp->file_desp == STDOUT_FILE_DESP)
	{
		// stdout_vt	
        #ifdef UART_MOD_ENABLE		
		   if((Put_Char(*in_buf_pos_ptr)) == NULL_CHAR)
		   {
		   	   system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		       Error_or_Warning_Proc("15.17.05", ERROR_OCCURED, system_status_flag); 
			   return EOF; 
		   }		   
		   ++num_chars_written;		   
		  #else
		     system_status_flag = ERR_MOD_NOT_ENABLED;
		     Error_or_Warning_Proc("15.17.06", ERROR_OCCURED, system_status_flag); 
		     return EOF; 
		  #endif
	}
	else
	{
	    for(num_chars_written = 0; num_chars_written < max_num_chars_to_write; ++num_chars_written)
	    {
	    	if((fp->stream_buf_cur_pos == fp->num_chars_stream_buf) || *in_buf_pos_ptr == NULL_CHAR)
		    {
		    	break;
		    }
		    fp->stream_buf[fp->stream_buf_cur_pos++] = *in_buf_pos_ptr++;		 
	    } 
	}
	memset(fp->stream_buf + stream_buf_before_proc_pos, NULL_CHAR, num_chars_written);
	fp->num_chars_stream_buf -= num_chars_written;	
    return num_chars_written;	
}

/*------------------------------------------------------------*
FUNCTION NAME  : File_Put_Str

DESCRIPTION    : put string into stream
								
INPUT          : 

OUTPUT         : 

NOTE           : File_Put_Str() operation is similiar to fputs()

Func ID        : 15.18 

Bugs           :  
-*------------------------------------------------------------*/
int16_t File_Put_Str(const char *const io_buf_ptr, file_t *const fp)
{
   const char *char_ptr = io_buf_ptr;
   uint8_t num_chars_written = 0;
  
	if(fp == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("15.18.01", ERROR_OCCURED, system_status_flag); 
		return EOF;
	}
    while(*char_ptr != NULL_CHAR)
	{
		if((File_Put_Char(*char_ptr, fp)) == EOF)
		{
			system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		    Error_or_Warning_Proc("15.18.02", ERROR_OCCURED, system_status_flag);            				
			break;
		}
		++char_ptr;
		++num_chars_written;
	}
	if(num_chars_written == 0)
	{
		return EOF;		
	}
	return (num_chars_written);
}


/*------------------------------------------------------------*
FUNCTION NAME  : VFile_Print

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : VFile_Print() operation is similar to vfprintf()

Func ID        : 15.19 

Bugs           :  
-*------------------------------------------------------------*/
int16_t VFile_Print(file_t *const fp, const char *const fmt, va_list *const print_va_list_ptr)
{
    int16_t num_chars_printed;
	    
    num_chars_printed = Print_Data(print_va_list_ptr, fp, fmt);
	if(num_chars_printed == EOF)
	{
		system_status_flag =  ERR_DEV_OPER_WRITE_FUNC;
		Error_or_Warning_Proc("15.19.01", ERROR_OCCURED, system_status_flag);
		return (EOF);
	}
	if(num_chars_printed == 0)
	{
		return 0;
	}
	if(fp->stream_buf_cur_pos + fp->num_chars_stream_buf + num_chars_printed + 1 >= MAX_STREAM_BUFFER_SIZE)
	{
		system_status_flag =  ERR_DEV_OPER_WRITE_FUNC;
		Error_or_Warning_Proc("15.19.02", ERROR_OCCURED, system_status_flag);
	    return (EOF);
	}
	fp->num_chars_stream_buf += (num_chars_printed);
	if (File_Put_Str((fp->stream_buf + fp->stream_buf_cur_pos), fp) == EOF)
	{
		system_status_flag =  ERR_DEV_OPER_WRITE_FUNC;
		Error_or_Warning_Proc("15.19.03", ERROR_OCCURED, system_status_flag);
	    return (EOF);
	}
	if(fp->file_desp == STDOUT_FILE_DESP)
	{
		if((File_Flush(fp)) != SUCCESS)
		{
			system_status_flag = ERR_FILE_FLUSH_PROC;
		    Error_or_Warning_Proc("15.22.15", ERROR_OCCURED, system_status_flag);
			return (EOF);
		}
	}		
    return (num_chars_printed);
}

/*------------------------------------------------------------*
FUNCTION NAME  : File_Print

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : File_Print() operation is similar to fprintf()

Func ID        : 15.20 

Bugs           :  
-*------------------------------------------------------------*/
int16_t File_Print(file_t *const fp, const char *const format_ptr, ...)
{
    va_list arg;
	 int16_t num_chars_printed;	
		
	if(fp == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.20.01", ERROR_OCCURED, system_status_flag);
	    return (EOF);		
	}
	if(format_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.20.02", ERROR_OCCURED, system_status_flag);
	    return (EOF);
	}
	if(*format_ptr == NULL_CHAR)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("15.20.03", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	va_start(arg, format_ptr); 
	num_chars_printed = VFile_Print(fp, format_ptr, &arg );
	va_end(arg);
	if(num_chars_printed == EOF)
	{
		system_status_flag =  ERR_DEV_OPER_WRITE_FUNC;
		Error_or_Warning_Proc("15.20.04", ERROR_OCCURED, system_status_flag);
	    return (EOF);
	}
	return num_chars_printed;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Str_Print

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : Str_Print() operation is similar to sprintf()
                 assuming sizeof(void *) == sizeof(int)   

Func ID        : 15.21 

Bugs           :  
-*------------------------------------------------------------*/
int16_t Str_Print(char *out_str, const char *const format_ptr, ...)
{
	file_t out_str_file;
	va_list arg;
	int16_t num_chars_printed;	
		
	if(out_str == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.21.01", ERROR_OCCURED, system_status_flag);
	    return (EOF);
	}
	if(format_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.21.02", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	if(*format_ptr == NULL_CHAR)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("15.21.03", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	File_Flush(&out_str_file);
	out_str_file.stream_type = STREAM_TYPE_NA;
	out_str_file.stream_io_type = STREAM_IO_OUTPUT;
	out_str_file.file_desp = STR_FILE_DESP;
	va_start(arg, format_ptr); 
	num_chars_printed = Print_Data(&arg, &out_str_file, format_ptr);
	va_end(arg);
	if(num_chars_printed == EOF)
	{
		system_status_flag =  ERR_DEV_OPER_WRITE_FUNC;
		Error_or_Warning_Proc("15.21.04", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	if(num_chars_printed == 0)
	{
		return 0;
	}
	memcpy(out_str, out_str_file.stream_buf, num_chars_printed);
	out_str[num_chars_printed + 1] = NULL_CHAR;
	return num_chars_printed;	
}

/*------------------------------------------------------------*
FUNCTION NAME  : Print_Data

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 15.22 

Bugs           :  
-*------------------------------------------------------------*/
static int16_t Print_Data(va_list *const print_va_list_ptr, file_t *const out_file_ptr, const char *const format_ptr)
{
	const char *transverse_ptr = format_ptr;
	char *print_str, *out_stream_buf_ptr;
	real_num_t real_num_print_data;
	uint32_t uint32_print_data; 
	int32_t int32_print_data; 
	uint16_t uint16_print_data; 
	int16_t int16_print_data, num_chars_printed = 0, ret_status;	
    uint8_t width_spec, pad_format, num_digits_after_point, proc_bit_field = 0; 
	char char_print_data;
	char char_in_str_arr[2];
    
    if(print_va_list_ptr == NULL_DATA_PTR || out_file_ptr == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.22.01", ERROR_OCCURED, system_status_flag);
		return EOF;
	}	
	out_stream_buf_ptr = out_file_ptr->stream_buf + out_file_ptr->stream_buf_cur_pos;	
	
	for (; *transverse_ptr != NULL_CHAR; ++transverse_ptr)
	{
		if (*transverse_ptr == '%')
		{
			++transverse_ptr;
			width_spec = pad_format = num_digits_after_point = 0;
			proc_bit_field &= ~(1 << 0); 
			if (*transverse_ptr == NULL_CHAR)
			{
				*out_stream_buf_ptr = NULL_CHAR;	            
				return num_chars_printed;
			}
			if (*transverse_ptr == '%')
			{
				if((Print_Char(&out_stream_buf_ptr, *transverse_ptr)) == NULL_CHAR)
				{
					*out_stream_buf_ptr = NULL_CHAR;
	                return num_chars_printed;
				}
			    ++num_chars_printed;
				continue;
			}
			if (*transverse_ptr == '-')
			{
				++transverse_ptr;
				pad_format = PAD_RIGHT;
			}
			while (*transverse_ptr == '0')
			{
				++transverse_ptr;
				pad_format |= PAD_ZERO;
			}
			for(; *transverse_ptr >= '0' && *transverse_ptr <= '9';	++transverse_ptr)
			{
			    width_spec *= 10;
			    width_spec += *transverse_ptr - '0';
			    if(width_spec > MAX_WIDTH_SPECIFIER)
			    {
			        width_spec = MAX_WIDTH_SPECIFIER;
			        for(;*transverse_ptr >= '0' && *transverse_ptr <= '9'; ++transverse_ptr);
				    break;
		    	}
			}
			
		    if(*transverse_ptr == '.')
			{
				++transverse_ptr;
				proc_bit_field |= (1 << 0); 
			}
			if((proc_bit_field & (1 << 0) ))
			{
				for(;*transverse_ptr >= '0' && *transverse_ptr <= '9';  ++transverse_ptr)	
			    {
			        num_digits_after_point *= 10;
			        num_digits_after_point += *transverse_ptr - '0';
			        if(num_digits_after_point > MAX_FLOAT_PREC_SPECIFIER)
			        {
			           num_digits_after_point = MAX_FLOAT_PREC_SPECIFIER;
			           for(;*transverse_ptr >= '0' && *transverse_ptr <= '9'; ++transverse_ptr);
				       break;
		        	}
		    	}	
			}
		    if(*transverse_ptr == 'c' )
			{
				/* char are converted to int then pushed on the stack */
				char_print_data = (char) va_arg(*print_va_list_ptr, int32_t);		
		       	char_in_str_arr[0] = char_print_data;
				char_in_str_arr[1] = NULL_CHAR;
				if((ret_status = Print_Str(&out_stream_buf_ptr, char_in_str_arr, width_spec, pad_format)) == 0)
				{
					 system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
					 Error_or_Warning_Proc("15.22.02", ERROR_OCCURED, system_status_flag);
					 *out_stream_buf_ptr = NULL_CHAR;
	                 return num_chars_printed;
				}
				num_chars_printed += ret_status;
				continue;
			}
			if(*transverse_ptr == 's')
			{		
				  print_str = va_arg(*print_va_list_ptr, char *);
				  if((ret_status = Print_Str(&out_stream_buf_ptr, print_str ? print_str: NULL_DATA_PTR, width_spec, pad_format)) == 0)
				  {
					   system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		               Error_or_Warning_Proc("15.22.03", ERROR_OCCURED, system_status_flag);
					   *out_stream_buf_ptr = NULL_CHAR;
	                   return num_chars_printed;
				  } 
				  num_chars_printed += ret_status;
				  continue;
			}
			if(*transverse_ptr == 'b' )
			{
				uint32_print_data = va_arg(*print_va_list_ptr, uint32_t); 
				if((ret_status = Print_Num(&out_stream_buf_ptr, uint32_print_data, width_spec, BASE_02 | STATE_NO << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_ALPHA_NA << ALPHA_BIT_POS)) == 0)
				{
					 system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		             Error_or_Warning_Proc("15.22.04", ERROR_OCCURED, system_status_flag);
					 *out_stream_buf_ptr = NULL_CHAR;
	                 return num_chars_printed;
				}
				num_chars_printed += ret_status;
				continue;
			}			
			if(*transverse_ptr == 'o' )
			{
				uint32_print_data = va_arg(*print_va_list_ptr, uint32_t); 
				if((ret_status = Print_Num(&out_stream_buf_ptr, uint32_print_data, width_spec, BASE_08 | STATE_NO << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_ALPHA_NA << ALPHA_BIT_POS)) == 0)
				{
					 system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
					 Error_or_Warning_Proc("15.22.05", ERROR_OCCURED, system_status_flag);
					 *out_stream_buf_ptr = NULL_CHAR;
	                 return num_chars_printed;
				}
				num_chars_printed += ret_status;
				continue;
			}
			if(*transverse_ptr == 'x')	
			{   
		        uint32_print_data = va_arg(*print_va_list_ptr, uint32_t); 
		        if((ret_status = Print_Num(&out_stream_buf_ptr, uint32_print_data, width_spec, BASE_16 | STATE_NO << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_SMALL_ALPHA << ALPHA_BIT_POS)) == 0)
				{
					 system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		             Error_or_Warning_Proc("15.22.06", ERROR_OCCURED, system_status_flag);
					 *out_stream_buf_ptr = NULL_CHAR;
	                 return num_chars_printed;
				}
				num_chars_printed += ret_status;
				continue;
			}
			if(*transverse_ptr == 'X')
			{	
		        uint32_print_data = va_arg(*print_va_list_ptr, uint32_t); 
		        if((ret_status = Print_Num(&out_stream_buf_ptr, uint32_print_data, width_spec, BASE_16 | STATE_NO << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_BIG_ALPHA << ALPHA_BIT_POS )) == 0)
				{
					 system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		             Error_or_Warning_Proc("15.22.07", ERROR_OCCURED, system_status_flag);
					 *out_stream_buf_ptr = NULL_CHAR;
	                 return num_chars_printed;
				}
				 num_chars_printed += ret_status;
				continue;
			}
			if(*transverse_ptr == 'h' )
			{
				++transverse_ptr;
			    switch(*transverse_ptr)
			    {
			        case 'u':
					   uint16_print_data = (uint16_t)va_arg(*print_va_list_ptr, uint32_t); 
				       if((ret_status = Print_Num(&out_stream_buf_ptr, uint16_print_data, width_spec, BASE_10 | STATE_NO << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_ALPHA_NA << ALPHA_BIT_POS)) == 0)
				       {
						   system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		                   Error_or_Warning_Proc("15.22.07", ERROR_OCCURED, system_status_flag);
						   *out_stream_buf_ptr = NULL_CHAR;
	                       return num_chars_printed;
				       }
					break;
					default:
				       int16_print_data = (int16_t)va_arg(*print_va_list_ptr, int32_t); 
				       if((ret_status = Print_Num(&out_stream_buf_ptr, int16_print_data, width_spec, BASE_10 | STATE_YES << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_ALPHA_NA << ALPHA_BIT_POS)) == 0)
				       {
						   system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		                   Error_or_Warning_Proc("15.22.08", ERROR_OCCURED, system_status_flag);
						   *out_stream_buf_ptr = NULL_CHAR;
	                       return num_chars_printed;
				       }
			           --transverse_ptr;
				}
				num_chars_printed += ret_status;
				continue;
			}
			if(*transverse_ptr == 'u' )
			{
				uint32_print_data = va_arg(*print_va_list_ptr, uint32_t); 
				if((ret_status = Print_Num(&out_stream_buf_ptr, uint32_print_data, width_spec, BASE_10 | STATE_NO << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_ALPHA_NA << ALPHA_BIT_POS)) == 0)
				{
					system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		            Error_or_Warning_Proc("15.22.09", ERROR_OCCURED, system_status_flag);
					*out_stream_buf_ptr = NULL_CHAR;
	                return num_chars_printed;
				}
				num_chars_printed += ret_status;
				continue;
			}
			if(*transverse_ptr == 'd' || *transverse_ptr == 'i')
			{	
                 int32_print_data = va_arg(*print_va_list_ptr, int32_t);		
                 if((ret_status = Print_Num(&out_stream_buf_ptr, int32_print_data, width_spec, BASE_10 | STATE_YES << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_ALPHA_NA << ALPHA_BIT_POS)) == 0)
				 {
					 system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		             Error_or_Warning_Proc("15.22.10", ERROR_OCCURED, system_status_flag);
					 *out_stream_buf_ptr = NULL_CHAR;
					 return num_chars_printed;
				 }					 
			  	 num_chars_printed += ret_status;
				 continue;
			}
			if(*transverse_ptr == 'f' )
			{
				real_num_print_data.float_num = (float)va_arg(*print_va_list_ptr, double); 
				if(((proc_bit_field & (1 << 0)) == 0))
				{
					//default max digits after point if '.'  operator for max precision spec is not specified
					num_digits_after_point = DEFAULT_FLOAT_PREC_WIDTH;
				}
				if((ret_status = Print_Float(&out_stream_buf_ptr, real_num_print_data, width_spec, num_digits_after_point, BASE_10 | STATE_YES << SIGN_FLAG_BIT_POS | pad_format << PAD_FORMAT_BIT_POS | NUM_CONV_ALPHA_NA << ALPHA_BIT_POS | FLOAT_FIXED_POINT_TYPE << REAL_NUM_TYPE_BIT_POS)) == 0)
				{
					system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		            Error_or_Warning_Proc("15.22.11", ERROR_OCCURED, system_status_flag);
		    		*out_stream_buf_ptr = NULL_CHAR; 
				   	return num_chars_printed;
				}
				num_chars_printed += ret_status;
				continue;
			}
		}
		else
		{
			if((Print_Char(&out_stream_buf_ptr, *transverse_ptr)) == NULL_CHAR)
			{
				system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		        Error_or_Warning_Proc("15.22.12", ERROR_OCCURED, system_status_flag);
				*out_stream_buf_ptr = NULL_CHAR;
				return num_chars_printed;
			}
			++num_chars_printed;
			if(*transverse_ptr == ENTER_CHAR)
			{
				if(out_file_ptr->file_desp == STDOUT_FILE_DESP)
				{
					if(out_file_ptr->stream_buf_cur_pos + out_file_ptr->num_chars_stream_buf + num_chars_printed + 1 >= MAX_STREAM_BUFFER_SIZE)
	                {
	                	system_status_flag =  ERR_DEV_OPER_WRITE_FUNC;
	                	Error_or_Warning_Proc("15.22.13", ERROR_OCCURED, system_status_flag);
	                    return (EOF);
	                }
	                out_file_ptr->num_chars_stream_buf += (num_chars_printed);
	                if(File_Put_Str((out_file_ptr->stream_buf + out_file_ptr->stream_buf_cur_pos), out_file_ptr) == EOF)
	                {
		               system_status_flag =  ERR_DEV_OPER_WRITE_FUNC;
		               Error_or_Warning_Proc("15.22.14", ERROR_OCCURED, system_status_flag);
	                   return (EOF);
	                }
   					if((File_Flush(out_file_ptr)) != SUCCESS)
					{
						system_status_flag = ERR_FILE_FLUSH_PROC;
		                Error_or_Warning_Proc("15.22.15", ERROR_OCCURED, system_status_flag);
				        *out_stream_buf_ptr = NULL_CHAR;
				        return num_chars_printed;
					}
					out_stream_buf_ptr = out_file_ptr->stream_buf + out_file_ptr->stream_buf_cur_pos;	
					num_chars_printed = 0;
				}
			}			
		}
	}
	*out_stream_buf_ptr = NULL_CHAR;	
	return num_chars_printed;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Print_Char

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           :   

Func ID        : 15.23 

Bugs           :  
-*------------------------------------------------------------*/
static char Print_Char(char **const out_str_ptr, const char print_char)
{
	char char_proc;
	
	if(out_str_ptr)
	{
		char_proc = print_char;
		**out_str_ptr = print_char;
		++(*out_str_ptr);
	}
	return char_proc;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Print_Str

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           :   

Func ID        : 15.24 

Bugs           :  
-*------------------------------------------------------------*/
static int16_t Print_Str(char **const out_str_ptr, const char *print_str, const uint8_t conf_width_spec, const uint8_t pad_format)
{
	int16_t num_chars_printed = 0;
	int8_t width_spec = conf_width_spec;
    char pad_char = ' ' ;
    const char *print_char_ptr = print_str;
    uint8_t str_len = 0;
	
	if(print_str == NULL_DATA_PTR)
	{
		system_status_flag =  ERR_NULL_PTR;
		Error_or_Warning_Proc("15.24.01", ERROR_OCCURED, system_status_flag);
		return 0;		
	}
	if (width_spec > 0)
	{
		str_len = strlen(print_str);
		if(str_len == 0)
		{
			system_status_flag =  ERR_DATA_OUT_OF_RANGE;
		    Error_or_Warning_Proc("15.24.01", ERROR_OCCURED, system_status_flag);
		    return 0;
		}
		if (str_len >= width_spec) 
		{
			width_spec = 0;
		}
		else 
		{ 
	       width_spec -= str_len;
		}
		if (pad_format & PAD_ZERO) 
		{ 
	        pad_char = '0';
		}
	}
	if (!(pad_format & PAD_RIGHT))
	{
		// pad_format left justified
		for ( ;width_spec > 0; --width_spec)
		{
			if((Print_Char(out_str_ptr, pad_char)) == NULL_CHAR)
			{
				return num_chars_printed;
			}
			++num_chars_printed;
		}
	}
	for ( ;*print_char_ptr; ++print_char_ptr)
	{
		if((Print_Char(out_str_ptr, *print_char_ptr)) == NULL_CHAR)
		{
			return num_chars_printed;
		}
		++num_chars_printed;
	}
	for ( ; width_spec > 0; --width_spec)
	{
		if((Print_Char(out_str_ptr, pad_char)) == NULL_CHAR)
		{
			return num_chars_printed;
		}
		++num_chars_printed;
	}
	return num_chars_printed;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Print_Num

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           :   

Func ID        : 15.25 

Bugs           :  
-*------------------------------------------------------------*/
static int16_t Print_Num(char **const out_str_ptr, const int32_t print_num, const uint8_t conf_width_spec, const uint16_t ctrl_flag )
{
	char print_buf[PRINT_BUF_LEN];
	char *cur_print_buf_ptr;
	uint32_t cur_unsign_print_num = (uint32_t)print_num;
	int32_t cur_digit; 
	int16_t num_chars_printed = 0, ret_status;
    char alpha_start_char;
	uint8_t num_negative_flag = STATE_NO, width_spec = conf_width_spec;
    const uint8_t base = ctrl_flag & 0x1F, sign_flag = (ctrl_flag >> SIGN_FLAG_BIT_POS) & 0x01, 
	   pad_format = (ctrl_flag >> PAD_FORMAT_BIT_POS) & 0x03, alpha_print_flag = (ctrl_flag >> ALPHA_BIT_POS ) & 0x03;
	
	if(print_num == 0)
	{
		print_buf[0] = '0';
		print_buf[1] = NULL_CHAR;
		return Print_Str(out_str_ptr, print_buf, width_spec, pad_format);
	}
	if(sign_flag == STATE_YES && base == BASE_10 && print_num < 0)
	{
		num_negative_flag = STATE_YES;
		cur_unsign_print_num = -print_num;
	}
	if(base > BASE_10)
	{
		switch(alpha_print_flag)
		{
			case NUM_CONV_BIG_ALPHA:
			   alpha_start_char = 'A';
			break;
			case NUM_CONV_SMALL_ALPHA:
			   alpha_start_char = 'a';
			break;
			default:
		    	system_status_flag =  ERR_FORMAT_INVALID;
		        Error_or_Warning_Proc("15.25.01", ERROR_OCCURED, system_status_flag);
		        return 0;
		}
	}
	cur_print_buf_ptr = print_buf + PRINT_BUF_LEN - 1;
	*cur_print_buf_ptr = NULL_CHAR;
	while(cur_unsign_print_num)
	{
		cur_digit = cur_unsign_print_num % base;
		if( cur_digit >= 10 )
		{
			cur_digit += alpha_start_char - '0' - 10;
		}
		*--cur_print_buf_ptr = cur_digit + '0';
		cur_unsign_print_num /= base;
	}
	if(num_negative_flag == STATE_YES)
	{
		if( width_spec && (pad_format & PAD_ZERO) ) 
		{
			if((Print_Char(out_str_ptr, '-')) == NULL_CHAR)
			{
				return num_chars_printed;
			}
			++num_chars_printed;
			--width_spec;
		}
		else
		{
			*--cur_print_buf_ptr = '-';
		}
	}
	if((ret_status = Print_Str(out_str_ptr, cur_print_buf_ptr, width_spec, pad_format)) == 0)
	{
		system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		Error_or_Warning_Proc("15.25.02", ERROR_OCCURED, system_status_flag);
		return num_chars_printed;		
	}
    return num_chars_printed + ret_status;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Print_Float

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           :   

Func ID        : 15.26 

Bugs           :  
-*------------------------------------------------------------*/
static int16_t Print_Float(char **const out_str_ptr, const real_num_t print_real_num, const uint8_t conf_width_spec, const uint8_t conf_num_digits_after_point, const uint16_t ctrl_flag )
{
	char print_buf[PRINT_BUF_LEN];
	int16_t num_chars_printed = 0, ret_status;
    uint8_t proc_bit_field = 0, width_spec = conf_width_spec, cur_digit;
    const uint8_t base = ctrl_flag & 0x1F, sign_flag = (ctrl_flag >> SIGN_FLAG_BIT_POS) & 0x01, 
	   pad_format = (ctrl_flag >> PAD_FORMAT_BIT_POS) & 0x03, alpha_print_flag = (ctrl_flag >> ALPHA_BIT_POS ) & 0x03, 
	   real_num_type = (ctrl_flag >> REAL_NUM_TYPE_BIT_POS) & 0x03;
	uint8_t num_digits_after_point = conf_num_digits_after_point;
	
	if(base != BASE_10)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("15.26.01", ERROR_OCCURED, system_status_flag);
		return 0;
	}
	memset(print_buf, NULL_CHAR, PRINT_BUF_LEN);		
	if((Real_Num_To_Str_Conv(print_buf, print_real_num, real_num_type, num_digits_after_point)) != SUCCESS)
	{
		system_status_flag =  ERR_DATA_CONV_PROC;
		Error_or_Warning_Proc("15.26.02", ERROR_OCCURED, system_status_flag);
		return 0;
	}
	ret_status = Print_Str(out_str_ptr, print_buf, width_spec, pad_format);
	if(ret_status == 0)
	{
		system_status_flag = ERR_DEV_OPER_WRITE_FUNC;
		Error_or_Warning_Proc("15.26.03", ERROR_OCCURED, system_status_flag);
		return num_chars_printed;
	}
   	return num_chars_printed + ret_status; 
}	

/*------------------------------------------------------------*
FUNCTION NAME  : Init_File

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 02.20 

Bugs           :  
-*------------------------------------------------------------*/
uint16_t Init_File(file_t *const file_ptr, const uint8_t file_desp, const uint8_t device_type, const uint8_t stream_type, const uint8_t stream_io_type)
{
	if(file_ptr == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("02.20.01", ERROR_OCCURED, system_status_flag);
		return system_status_flag; 
	}
	if(device_type >= DEVICE_NA)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("02.20.02", ERROR_OCCURED, system_status_flag);
		return system_status_flag; 		
	}
	if(stream_type >= STREAM_TYPE_NA)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("02.20.03", ERROR_OCCURED, system_status_flag);
		return system_status_flag; 
	}
	if(stream_io_type >= STREAM_IO_NA)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("02.20.04", ERROR_OCCURED, system_status_flag);
		return system_status_flag; 
	}
	if((File_Flush(file_ptr)) != SUCCESS)
	{
		system_status_flag = ERR_FILE_FLUSH_PROC;
		Error_or_Warning_Proc("02.20.05", ERROR_OCCURED, system_status_flag);
		return system_status_flag;
	}
	file_ptr->device_type = device_type;
	file_ptr->stream_type = stream_type;
	file_ptr->stream_io_type = stream_io_type;
	file_ptr->file_desp = file_desp;
	return SUCCESS;		
}

/*------------------------------------------------------------*
FUNCTION NAME  : File_Flush

DESCRIPTION    : File_Flush() operation is similar to fflush()
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 02.21 

Bugs           :  
-*------------------------------------------------------------*/
uint16_t File_Flush(file_t *const file_ptr)
{
	if(file_ptr == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("02.21.01", ERROR_OCCURED, system_status_flag);
		return EOF;
	}
	memset(file_ptr->stream_buf, NULL_CHAR, MAX_STREAM_BUFFER_SIZE + 1);
	file_ptr->num_chars_stream_buf = 0;
	file_ptr->stream_buf_cur_pos = 0;
	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : strtoq

DESCRIPTION    : converts the string pointed to by nptr to a 64-bit, long-long integer representation. 
        This function recognizes (in order) an optional string of spaces, an optional sign, an optional base indicator (0 for octal, X or x for hexadecimal),
        and a string of digits. The first unrecognized character ends the string. A pointer to this unrecognized character is stored in the object addressed by endptr, 
       if endptr is not NULL.

       If base is non-zero, its value determines the set of recognized digits and overrides the optional base indicator character. 
        If base is zero, nptr is assumed to be base 10, unless an optional base indicator character is given
								
INPUT          : nptr - Points to a character string for strtoq() to convert.
                 endptr - Is a result parameter that, if not NULL, returns a string beginning with the first character that strtoq() does not attempt to convert. 
                 base  - Is the base of the string, a value between 0 and 36.
				 
OUTPUT         : returns the converted value, if there is any. If no conversion was performed, strtoq() returns a zero. 
                 If the converted value overflows, strtoq() returns QUAD_MAX or QUAD_MIN (according to the sign of the value) 

NOTE           : 

Func ID        : 02.22 

Bugs           :  
-*------------------------------------------------------------*/
quad_t strtoq(const char *const nptr, char **endptr, uint8_t base)
{
	const char *s;
	u_quad_t acc;
	unsigned char c;
	u_quad_t qbase, cutoff;
	int neg, any, cutlim;

	if(nptr == NULL_DATA_PTR) 
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("02.22.01", ERROR_OCCURED, system_status_flag);
		return 0;
	}
	/*
	 * Skip white space and pick up leading +/- sign if any.
	 * If base is 0, allow 0x for hex and 0 for octal, else
	 * assume decimal; if base is already 16, allow 0x.
	 */
	s = nptr;
	do 
	{
		c = *s++;
	} while (isspace(c));
	if (c == '-')
	{
		neg = 1;
		c = *s++;
	} 
	else
	{
		neg = 0;
		if (c == '+')
			c = *s++;
	}
	if ((base == 0 || base == BASE_16) &&
	    c == '0' && (*s == 'x' || *s == 'X'))
	{
		c = s[1];
		s += 2;
		base = BASE_16;
	}
	if (base == 0)
		base = c == '0' ? BASE_08 : BASE_10;

	/*
	 * Compute the cutoff value between legal numbers and illegal
	 * numbers.  That is the largest legal value, divided by the
	 * base.  An input number that is greater than this value, if
	 * followed by a legal input character, is too big.  One that
	 * is equal to this value may be valid or not; the limit
	 * between valid and invalid numbers is then based on the last
	 * digit.  For instance, if the range for quads is
	 * [-9223372036854775808..9223372036854775807] and the input base
	 * is 10, cutoff will be set to 922337203685477580 and cutlim to
	 * either 7 (neg==0) or 8 (neg==1), meaning that if we have
	 * accumulated a value > 922337203685477580, or equal but the
	 * next digit is > 7 (or 8), the number is too big, and we will
	 * return a range error.
	 *
	 * Set any if any `digits' consumed; make it negative to indicate
	 * overflow.
	 */
	qbase = (unsigned)base;
	cutoff = neg ? (u_quad_t)-(QUAD_MIN + QUAD_MAX) + QUAD_MAX : QUAD_MAX;
	cutlim = cutoff % qbase;
	cutoff /= qbase;
	for (acc = 0, any = 0;; c = *s++)
	{
		if ((Char_Is_ASCII(c)) != STATE_YES)
			break;
		if (isdigit(c))
			c -= '0';
		else if (isalpha(c))
			c -= isupper(c) ? 'A' - 10 : 'a' - 10;
		else
			break;
		if (c >= base)
			break;
		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim))
		{
			any = -1;
		}
		else
		{
			any = 1;
			acc *= qbase;
			acc += c;
		}
	}
	if (any < 0)
	{
		acc = neg ? QUAD_MIN : QUAD_MAX;
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
	    Error_or_Warning_Proc("02.22.02", WARNING_OCCURED, system_status_flag);
	} 
	else if (neg)
		acc = -acc;
	if (endptr != 0)
		*((const char **)endptr) = any ? s - 1 : nptr;
	return (acc);
}


 /*------------------------------------------------------------*
FUNCTION NAME  : strtoul

DESCRIPTION    :  converts the string pointed to by nptr to a 64-bit, unsigned-long integer representation. 
        This function recognizes (in order) an optional string of spaces, an optional sign, an optional base indicator (0 for octal, X or x for hexadecimal),
        and a string of digits. The first unrecognized character ends the string. A pointer to this unrecognized character is stored in the object addressed by endptr, 
       if endptr is not NULL.

       If base is non-zero, its value determines the set of recognized digits and overrides the optional base indicator character. 
        If base is zero, nptr is assumed to be base 10, unless an optional base indicator character is given
								
INPUT          : nptr - points to a sequence of characters that can be interpreted as a numeric value of type unsigned long int.
                 endptr - Is a result parameter that, if not NULL, returns a string beginning with the first character that strtoul() does not attempt to convert. 
                 base  - Is the base of the string, a value between 0 and 36.
				 
OUTPUT         : returns the converted value, if there is any. If no conversion was performed, strtoul() returns a zero. 
                 If the converted value overflows, strtoul() returns ULONG_MAX. 

NOTE           : 

Func ID        : 02.23 

Bugs           :  
-*------------------------------------------------------------*/
unsigned long strtoul(const char *const nptr, char **endptr, uint8_t base)
{
	const char *s;
	unsigned long acc, cutoff;
	int c;
	int neg, any, cutlim;
	
	if(nptr == NULL_DATA_PTR) 
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("02.23.01", ERROR_OCCURED, system_status_flag);
		return 0;
	}
	/*
	 * Skip white space and pick up leading +/- sign if any.
	 * If base is 0, allow 0x for hex and 0 for octal, else
	 * assume decimal; if base is already 16, allow 0x.
	 */
	s = nptr;
	do {
		c = (unsigned char) *s++;
	} while (isspace(c));
	if (c == '-')
	{
		neg = 1;
		c = *s++;
	}
	else
	{
		neg = 0;
		if (c == '+')
			c = *s++;
	}
	if ((base == 0 || base == BASE_16) &&
	    c == '0' && (*s == 'x' || *s == 'X'))
	{
		c = s[1];
		s += 2;
		base = BASE_16;
	}
	if (base == 0)
		base = c == '0' ? BASE_08: BASE_10;
	cutoff = ULONG_MAX / (unsigned long)base;
	cutlim = ULONG_MAX % (unsigned long)base;
	for (acc = 0, any = 0;; c = (unsigned char) *s++)
	{
		if (isdigit(c))
			c -= '0';
		else if (isalpha(c))
			c -= isupper(c) ? 'A' - 10 : 'a' - 10;
		else
			break;
		if (c >= base)
			break;
		if (any < 0)
			continue;
		if (acc > cutoff || (acc == cutoff && c > cutlim))
		{
			any = -1;
			acc = ULONG_MAX;
			system_status_flag = ERR_DATA_OUT_OF_RANGE;
	    	Error_or_Warning_Proc("02.23.02", WARNING_OCCURED, system_status_flag);
		} 
		else
		{
			any = 1;
			acc *= (unsigned long)base;
			acc += c;
		}
	}
	if (neg && any > 0)
		acc = -acc;
	if (endptr != 0)
		*endptr = (char *) (any ? s - 1 : nptr);
	return (acc);
}

/*------------------------------------------------------------*
FUNCTION NAME  : Char_Is_ASCII

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 02.28 

Bugs           :  
-*------------------------------------------------------------*/
uint16_t Char_Is_ASCII(const unsigned char test_char)
{
	if(test_char > ASCII_MAX_CHAR_CODE)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("02.28.01", ERROR_OCCURED, system_status_flag);
		return system_status_flag;
	}
	if(test_char >= NUM_0_CHAR && test_char <= NUM_9_CHAR)
	{
		return STATE_YES;
	}
	if(test_char >= ENGLISH_SMALL_ALPHA_a_CHAR && test_char <= ENGLISH_SMALL_ALPHA_z_CHAR)
	{
		return STATE_YES;
	}
	if(test_char >= ENGLISH_BIG_ALPHA_A_CHAR && test_char <= ENGLISH_BIG_ALPHA_Z_CHAR)
	{
		return STATE_YES;
	}
	if(test_char >= BEGIN_CTRL_CHARS_ASCII_CODE && test_char <= END_CTRL_CHARS_ASCII_CODE)
	{
		if(isspace(test_char))
		{
		   return STATE_YES;
		}
		else
		{
			return STATE_NO;
		}
	}
	return STATE_YES;
}	

/*------------------------------------------------------------*
FUNCTION NAME  : Error_or_Warning_Proc

DESCRIPTION    : 
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 02.14  

BUGS           :              
-*------------------------------------------------------------*/
uint8_t Error_or_Warning_Proc(const char *const error_trace_str, const uint8_t warn_or_error_format, const uint32_t warning_or_error_code)
{
	printf("\n");
	switch(warn_or_error_format)
	{
		case WARNING_OCCURED:
			printf("WRN: "); 
		break;
		default:
		   printf("ERR: ");	
	}
	printf("%s, code: %u \n", error_trace_str, warning_or_error_code);
	return SUCCESS;
}

/*------------------------------------------------------------*
FUNCTION NAME  : Power_Of

DESCRIPTION    :
								
INPUT          :

OUTPUT         : 

NOTE           : 

Func ID        : 02.10 

Bugs           :   
-*------------------------------------------------------------*/
double Power_Of(const uint8_t base, const int16_t power)
{
    double power_val = 1.0;
	uint8_t i = 0, uint8_power;
  
    if(power == 0)
    {
       return power_val;
    }
	if(power < 0)
	{
		uint8_power = (uint8_t)((-power));		
	}
	else
	{
		uint8_power = (uint8_t)power;		
	}
    for(i = 1; i <= uint8_power; ++i)
    {
        power_val *= base;
    }
	if(power > 0)
	{
		return power_val;
	}
	return (1 / power_val);
}

/*------------------------------------------------------------*
FUNCTION NAME  : Reverse_Str_Use_Same_Str

DESCRIPTION    : reverses a string 'str' of length 'len'
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 02.27 

Bugs           :  
-*------------------------------------------------------------*/
uint16_t Reverse_Str_Use_Same_Str(char *const str, const uint8_t len) 
{ 
    uint8_t i, j;
    char temp;
 	
	if(str == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("02.27.01", ERROR_OCCURED, system_status_flag);
		return system_status_flag;  
	}
	i = strlen(str);
	if(i == 0)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("02.27.02", ERROR_OCCURED, system_status_flag);
		return system_status_flag; 
	}
	if(len > i)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("02.27.03", ERROR_OCCURED, system_status_flag);
		return system_status_flag; 
	}
	i = 0;
	j = len - 1;
    while (i < j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        ++i; 
		--j; 
    } 
	return SUCCESS;
} 	

/*------------------------------------------------------------*
FUNCTION NAME  : Get_Num_Digits_Based_Data

DESCRIPTION    :
								
INPUT          : 

OUTPUT         : 

NOTE           : 

Func ID        : 02.26  

BUGS           :  
-*------------------------------------------------------------*/
uint16_t Get_Num_Digits_Based_Data(const uint32_t num, uint32_t *const num_digits_data_ptr, const uint8_t base, const uint8_t get_num_digits_type)
{
	uint32_t power_base_cur_digit;
	uint16_t max_num_digits;
    int16_t cur_num_digits; 
	
	if(num_digits_data_ptr == NULL_DATA_PTR)
	{
		system_status_flag = ERR_NULL_PTR;
		Error_or_Warning_Proc("02.26.01", ERROR_OCCURED, system_status_flag);
		return system_status_flag;  
	}
	*num_digits_data_ptr = 0;
	if(base > BASE_16)
	{
		system_status_flag = ERR_DATA_OUT_OF_RANGE;
		Error_or_Warning_Proc("02.26.02", ERROR_OCCURED, system_status_flag);
		return system_status_flag;  
	}
	switch(get_num_digits_type)
	{
		case GET_NUM_DIGITS:
		case GET_POWER_OF_CUR_NUM_DIGITS:
		case GET_POWER_OF_NEXT_NUM_DIGITS:
		break;
		default:
		  system_status_flag = ERR_FORMAT_INVALID;
		  Error_or_Warning_Proc("02.26.03", ERROR_OCCURED, system_status_flag);
		  return system_status_flag;  
	}
	max_num_digits = sizeof(uint32_t) * 8;
	switch(base)
	{
		case BASE_08:
		   max_num_digits = ((sizeof(uint32_t) * 8) / 3) + 1;
		break;
        case BASE_16:
            max_num_digits = ((sizeof(uint32_t) * 8) / 4);
		break;
        case BASE_10: 
           max_num_digits = (sizeof(uint32_t) * 3);
        break; 
	}
	for(cur_num_digits = (int16_t)max_num_digits; cur_num_digits >= 0; --cur_num_digits )
	{
		power_base_cur_digit = (uint32_t)Power_Of(base, (uint8_t)cur_num_digits);
		if((num / power_base_cur_digit) != 0)
		{
			switch(get_num_digits_type)
			{
				case GET_NUM_DIGITS:
				  *num_digits_data_ptr = cur_num_digits + 1; 
				break;
                case GET_POWER_OF_CUR_NUM_DIGITS:
				  *num_digits_data_ptr = power_base_cur_digit;
                break;	
                case GET_POWER_OF_NEXT_NUM_DIGITS:
				  *num_digits_data_ptr = power_base_cur_digit * base;
                break;                               				
			}
			break;
		}
	}
    return SUCCESS;
}
