
/* Copyright (c) 2014, Linear Technology Corp.(LTC)
All rights reserved.

Linear Technology Confidential - For Customer Use Only

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */


#ifndef LTC2937_H_
#define LTC2937_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "LTC2937_reg_defs.h"
#include <stdint.h>
#include <stddef.h>


// Type declarations 
/* Functions meeting the types of  smbus_write_register and smbus_read_register must be provided to 
this API.  They will implement the SMBus read and write transactions on your hardware. If the register size of LTC2937 is 8 bits,
functions should be provided that implement SMBus read byte and write byte. If the register size of LTC2937 is 16 bits,
functions should be provided that implement SMBus read word and write word. smbus_read_register needs to store 
the value read from the LTC2937 into the variable data. Both functions should return 0 on success and a non-0 error code on failure.
The API functions will return your error codes on failure and a 0 on success*/
typedef struct {
  int file_descriptor;
} port_configuration_t;
typedef int (*smbus_write_register)(uint8_t addr,uint8_t command_code, uint16_t data, port_configuration_t *port_configuration);
typedef int (*smbus_read_register)(uint8_t addr,uint8_t command_code, uint16_t *data, port_configuration_t *port_configuration);
typedef void* LTC2937;
typedef struct {
  uint8_t addr;
  smbus_read_register read_register;
  smbus_write_register write_register;
  port_configuration_t *port_configuration;
} LTC2937_chip_cfg_t;

// function declarations
LTC2937 LTC2937_init(LTC2937_chip_cfg_t *cfg);
uint32_t LTC2937_read_modify_write_register(LTC2937 chip_handle, uint16_t registerinfo, uint16_t data);
uint32_t LTC2937_read_register(LTC2937 chip_handle, uint16_t registerinfo, uint16_t *data);

/* Multiple LTC2937 use.  
  Multiple LTC2937s can be used with this API.  Each one must be initialized.  The LTC2937_init requires some memory for each
  LTC2937.  This memory can be statically allocated by defining MAX_NUM_LTC2937_INSTANCES to the number of LTC2937s required.  
  Alternatively it can be dynamically allocated by defining LTC2937_USE_MALLOC.  The default is to not use malloc and statically 
  allocate one LTC2937.
  */
#ifndef MAX_NUM_LTC2937_INSTANCES
#define MAX_NUM_LTC2937_INSTANCES 1
#endif
#ifdef __cplusplus
}
#endif
#endif /* LTC2937_H_ */
