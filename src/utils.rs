use rp_pico::hal::pio::Tx;

pub struct Texture {
  pub base: &'static[u8],
  pub alpha: Option<&'static[u8]>,
  pub width: usize,
  pub height: usize
}

pub static a: [u8; 2] = [0x00, 0x00];
pub static b: [u8; 2] = [0x00, 0x00];

// pub const texture_test: Texture = Texture {
//   base: &b,
//   alpha: &a,
//   height: 2,
//   width: 2
// };

