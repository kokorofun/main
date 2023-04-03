use crate::utils::Texture;
use cortex_m::delay::Delay;
use rp_pico::{hal::{spi::{self, Disabled, Enabled}, gpio::{PushPull, bank0::Gpio5, Pin, Output}, self}, pac::{SPI0, self}};
use rp_pico::hal::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use cortex_m::prelude::_embedded_hal_blocking_spi_Write;
use cortex_m::prelude::_embedded_hal_spi_FullDuplex;


use micromath::F32Ext;

use crate::REVERSE_LOOKUP;

type Spi = spi::Spi<Enabled, SPI0, 8>;
type SpiCs = Pin<Gpio5, Output<PushPull>>;


pub struct Display {
  buffer: [[u8; 52]; 240],
  to_update: [bool; 240],
  spi: Spi,
  spi_cs: SpiCs,
  delay: Delay,
  vcom: u8
}

pub struct Position {
  pub x: usize,
  pub y: usize
}

impl Display {
  pub fn new(spi: Spi, spi_cs: SpiCs, delay: Delay) -> Self {
    let mut buf = [[0x00; 52]; 240];
    for line in 0..240 {
      buf[line][0] = REVERSE_LOOKUP[239 - line];
    }

    let mut s = Self {
      buffer: buf,
      to_update: [false; 240],
      spi,
      spi_cs,
      delay,
      vcom: 0x00
    };

    s.clear();

    s
  }

  pub fn clear(&mut self) {
    self.spi_cs.set_high().unwrap();
    self.delay.delay_us(20);

    self.spi.write(&[0b00100000 | self.vcom, 0x00]).unwrap();

    self.delay.delay_us(20);
    self.spi_cs.set_low().unwrap();
  }

  pub fn update(&mut self) {

    // self.vcom = !self.vcom & 0b01000000;
    self.vcom = !0x00 & 0b01000000;

    self.spi_cs.set_high().unwrap();
    self.delay.delay_us(50);

    self.spi.send(0b10000000_u8 | self.vcom).unwrap();

    for line in 0..240 {

      if !self.to_update[239 - line] {
        continue;
      }

      let mut data: [u8; 52] = [0xFF; 52];
      data[0] = REVERSE_LOOKUP[line + 1];
      data[51] = 0x00;

      for j in 1..51 {
        // data[ 51 - j] = !REVERSE_LOOKUP[self.buffer[239 - line][j-1] as usize];
        data[ 51 - j] = !REVERSE_LOOKUP[self.buffer[239 - line][j-1 + 1] as usize];
      }


      // self.spi.write(&self.buffer[line]).unwrap();
      self.spi.write(&data).unwrap();
    }
    self.delay.delay_us(5);

    self.spi.send(0x00).unwrap();

    self.delay.delay_us(50);
    self.spi_cs.set_low().unwrap();

    self.to_update = [false; 240];


    // self.delay.delay_ms(66);


  }

  // pub fn draw_texture(&mut self, pos: Position, base: &[u8], alpha: &[u8], width: usize, height: usize) {
    pub fn draw_texture(&mut self, pos: Position, texture: &Texture) {

    let bytes_per_line = (texture.width as f32 / 8.).ceil() as usize;
    let final_pixels = bytes_per_line*8 - texture.width;


    let start_col = pos.x / 8;
    let shift = pos.x % 8;

    for line in pos.y..(texture.height as usize + pos.y as usize) {
      for col in 0..bytes_per_line {
        let idx = bytes_per_line * (line - pos.y) + col;
        let mut alpha = match texture.alpha {
          None => 0x00,
          Some(val) => val[idx]
        };

        if col == bytes_per_line - 1 {
          alpha = alpha | (0xff >> (8 - 1 - final_pixels));
        }

        let val = texture.base[ idx ];
        if shift != 0 {
          let final_val = (val >> shift) | self.buffer[line][col + start_col + 1] & ((alpha >> shift) | (0xff << (8 - shift)));
          let target = self.buffer[line][col + start_col + 1];
          self.buffer[line][col + start_col + 1] = final_val;
          if !self.to_update[line] && target != final_val {
            self.to_update[line] = true;
          }


          let final_val = (val << (8 - shift)) | self.buffer[line][col + start_col + 1 + 1] & (alpha << (8 - shift) | (0xff >> shift) );
          let target = self.buffer[line][col + start_col + 1 + 1];
          self.buffer[line][col + start_col + 1 + 1] = final_val;
          if !self.to_update[line] && target != final_val {
            self.to_update[line] = true;
          }
        } else {
          let final_val = val | (self.buffer[line][col + start_col + 1] & alpha);
          let target = self.buffer[line][col + start_col + 1];
          self.buffer[line][col + start_col + 1] = final_val;
          if !self.to_update[line] && target != final_val {
            self.to_update[line] = true;
          }
        }
      }
    }
  }
}