
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



#include "LTC2937.h"

#ifndef LTC2937_USE_MALLOC
int LTC2937_instances = 0;
LTC2937_chip_cfg_t LTC2937_chip_array[MAX_NUM_LTC2937_INSTANCES]; 
#endif


//private function
LTC2937 LTC2937_alloc(void) {
  // this function "allocates" a LTC2937_chip structure. It may or may not use malloc.
#ifdef LTC2937_USE_MALLOC
  return malloc(sizeof(LTC2937_chip_cfg_t));
#else
  if (LTC2937_instances < MAX_NUM_LTC2937_INSTANCES) {
    return &LTC2937_chip_array[LTC2937_instances++];
  } else {
  return 0;
}
#endif
}

LTC2937 LTC2937_init(LTC2937_chip_cfg_t *cfg) {
  LTC2937_chip_cfg_t *chip = LTC2937_alloc();
  if (chip == NULL) return NULL;
  chip->addr = cfg->addr;
  chip->write_register = cfg->write_register;
  chip->read_register = cfg->read_register;
  chip->port_configuration = cfg->port_configuration;
  return (LTC2937) chip;
}

static inline uint8_t get_size(uint16_t registerinfo) {
  return (registerinfo >> 8) & 0x0F;
}
static inline uint8_t get_subaddr(uint16_t registerinfo) {
  return (registerinfo) & 0xFF;
}
static inline uint16_t get_offset(uint16_t registerinfo) {
  return (registerinfo >> 12) & 0x0F;
}
static inline int get_mask(uint16_t registerinfo) {
  int mask = 1 << get_offset(registerinfo);
  int size = get_size(registerinfo);
  if (size == 0) {
    return (1<<16)-1; //adressing whole command code
  }
  int i;
  for (i=0; i<size-1; i++){
    mask |= mask << 1;
  }
  return mask;
}

uint32_t LTC2937_read_modify_write_register(LTC2937 chip_handle, uint16_t registerinfo, uint16_t data) {
  LTC2937_chip_cfg_t *chip = (LTC2937_chip_cfg_t *) chip_handle;
  int failure;
  uint8_t command_code = get_subaddr(registerinfo);
  uint8_t offset = get_offset(registerinfo);
  uint16_t mask = get_mask(registerinfo);
  uint16_t read_data;
  failure = chip->read_register(chip->addr,command_code,&read_data,chip->port_configuration);
  if (failure) return failure;
  data = (read_data & ~mask) | (data << offset);
  return chip->write_register(chip->addr,command_code,data,chip->port_configuration);
}

uint32_t LTC2937_read_register(LTC2937 chip_handle, uint16_t registerinfo, uint16_t *data) {
  LTC2937_chip_cfg_t *chip = (LTC2937_chip_cfg_t *) chip_handle;
  int result;
  uint8_t command_code = get_subaddr(registerinfo);
  uint8_t offset = get_offset(registerinfo);
  uint16_t mask = get_mask(registerinfo);
  result = chip->read_register(chip->addr,command_code,data,chip->port_configuration);
  *data &= mask;
  *data = *data >> offset;
  return result;
}
