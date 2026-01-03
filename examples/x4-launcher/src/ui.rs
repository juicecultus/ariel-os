use embedded_graphics::{
    mono_font::{
        ascii::{FONT_10X20, FONT_6X10},
        MonoTextStyle,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{
        Line,
        PrimitiveStyle,
        PrimitiveStyleBuilder,
        Rectangle,
    },
    text::{Baseline, Text},
};

use crate::ssd1677;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Rotation {
    Portrait,
    Landscape,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    List,
    Grid,
}

pub struct UiStatus<'a> {
    pub title: &'a str,
    pub battery_pct: Option<u8>,
    pub sd_present: bool,
    pub wifi_on: bool,
    pub bt_on: bool,
}

pub struct UiNavHints<'a> {
    pub left: &'a str,
    pub ok: &'a str,
    pub right: &'a str,
    pub back: &'a str,
}

pub struct UiLayout {
    pub full: Rectangle,
    pub status: Rectangle,
    pub content: Rectangle,
    pub nav: Rectangle,
}

pub fn layout(rotation: Rotation) -> UiLayout {
    let (w, h) = match rotation {
        Rotation::Portrait => (ssd1677::DISPLAY_WIDTH as i32, ssd1677::DISPLAY_HEIGHT as i32),
        Rotation::Landscape => (ssd1677::DISPLAY_HEIGHT as i32, ssd1677::DISPLAY_WIDTH as i32),
    };

    let full = Rectangle::new(Point::new(0, 0), Size::new(w as u32, h as u32));

    let status_h: i32 = 52;
    let nav_h: i32 = 64;

    let status = Rectangle::new(Point::new(0, 0), Size::new(w as u32, status_h as u32));
    let nav = Rectangle::new(
        Point::new(0, h - nav_h),
        Size::new(w as u32, nav_h as u32),
    );
    let content = Rectangle::new(
        Point::new(0, status_h),
        Size::new(w as u32, (h - status_h - nav_h) as u32),
    );

    UiLayout {
        full,
        status,
        content,
        nav,
    }
}

pub struct UiCanvas<'a, D> {
    inner: &'a mut ssd1677::Ssd1677<D>,
    rotation: Rotation,
}

impl<'a, D> UiCanvas<'a, D>
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    pub fn new(inner: &'a mut ssd1677::Ssd1677<D>, rotation: Rotation) -> Self {
        Self { inner, rotation }
    }

    pub async fn flush_refresh(&mut self, mode: ssd1677::RefreshMode) {
        self.inner.flush().await;
        self.inner.refresh(mode).await;
    }

    fn map_point(rotation: Rotation, p: Point) -> Option<Point> {
        match rotation {
            Rotation::Portrait => {
                if p.x < 0
                    || p.y < 0
                    || p.x >= ssd1677::DISPLAY_WIDTH as i32
                    || p.y >= ssd1677::DISPLAY_HEIGHT as i32
                {
                    None
                } else {
                    Some(p)
                }
            }
            Rotation::Landscape => {
                let w = ssd1677::DISPLAY_HEIGHT as i32;
                let h = ssd1677::DISPLAY_WIDTH as i32;
                if p.x < 0 || p.y < 0 || p.x >= w || p.y >= h {
                    return None;
                }
                let x_p = p.y;
                let y_p = (ssd1677::DISPLAY_HEIGHT as i32 - 1) - p.x;
                Some(Point::new(x_p, y_p))
            }
        }
    }
}

impl<'a, D> OriginDimensions for UiCanvas<'a, D>
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    fn size(&self) -> Size {
        match self.rotation {
            Rotation::Portrait => Size::new(ssd1677::DISPLAY_WIDTH as u32, ssd1677::DISPLAY_HEIGHT as u32),
            Rotation::Landscape => Size::new(ssd1677::DISPLAY_HEIGHT as u32, ssd1677::DISPLAY_WIDTH as u32),
        }
    }
}

impl<'a, D> DrawTarget for UiCanvas<'a, D>
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let rotation = self.rotation;
        self.inner.draw_iter(
            pixels
                .into_iter()
                .filter_map(|Pixel(p, c)| Self::map_point(rotation, p).map(|p2| Pixel(p2, c))),
        )
    }
}

pub fn clear<D>(canvas: &mut UiCanvas<'_, D>)
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    canvas
        .bounding_box()
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(canvas)
        .ok();
}

fn draw_hline<D>(canvas: &mut UiCanvas<'_, D>, y: i32)
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    let bb = canvas.bounding_box();
    Line::new(
        Point::new(bb.top_left.x, y),
        Point::new(bb.bottom_right().unwrap().x - 1, y),
    )
    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
    .draw(canvas)
    .ok();
}

fn draw_battery_icon<D>(canvas: &mut UiCanvas<'_, D>, top_right: Point, pct: Option<u8>)
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    let w = 28;
    let h = 14;
    let nub_w = 3;
    let nub_h = 8;

    let outer = Rectangle::new(Point::new(top_right.x - w, top_right.y), Size::new(w as u32, h as u32));
    outer
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(canvas)
        .ok();

    let nub = Rectangle::new(
        Point::new(top_right.x - 1, top_right.y + (h - nub_h) / 2),
        Size::new(nub_w as u32, nub_h as u32),
    );
    nub.into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(canvas)
        .ok();

    if let Some(p) = pct {
        let p = p.min(100);
        let inner_w = ((w - 4) as u32 * p as u32) / 100;
        let inner = Rectangle::new(
            Point::new(outer.top_left.x + 2, outer.top_left.y + 2),
            Size::new(inner_w, (h - 4) as u32),
        );
        inner
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(canvas)
            .ok();
    }
}

fn draw_toggle_icon<D>(canvas: &mut UiCanvas<'_, D>, p: Point, label: &str, on: bool)
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    let box_w = (label.len() as i32 * 6) + 10;
    let box_h = 14;

    let r = Rectangle::new(p, Size::new(box_w as u32, box_h as u32));
    r.into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(canvas)
        .ok();

    if on {
        let fill = Rectangle::new(Point::new(p.x + 1, p.y + 1), Size::new((box_w - 2) as u32, (box_h - 2) as u32));
        fill.into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(canvas)
            .ok();
        let inv_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::Off);
        Text::with_baseline(label, Point::new(p.x + 5, p.y + 11), inv_style, Baseline::Alphabetic)
            .draw(canvas)
            .ok();
    } else {
        Text::with_baseline(label, Point::new(p.x + 5, p.y + 11), style, Baseline::Alphabetic)
            .draw(canvas)
            .ok();
    }
}

pub fn draw_chrome<D>(canvas: &mut UiCanvas<'_, D>, l: &UiLayout, status: UiStatus<'_>, nav: UiNavHints<'_>)
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    draw_hline(canvas, l.status.bottom_right().unwrap().y - 1);
    draw_hline(canvas, l.nav.top_left.y);

    let title_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    Text::new(status.title, Point::new(16, 32), title_style)
        .draw(canvas)
        .ok();

    let mut x = canvas.bounding_box().bottom_right().unwrap().x - 16;

    draw_battery_icon(canvas, Point::new(x, 18), status.battery_pct);
    x -= 36;

    draw_toggle_icon(canvas, Point::new(x - 44, 18), "SD", status.sd_present);
    x -= 54;

    draw_toggle_icon(canvas, Point::new(x - 44, 18), "WiFi", status.wifi_on);
    x -= 62;

    draw_toggle_icon(canvas, Point::new(x - 38, 18), "BT", status.bt_on);

    let nav_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let y = l.nav.top_left.y + 24;
    Text::new(nav.back, Point::new(16, y), nav_style)
        .draw(canvas)
        .ok();

    let mid = (canvas.bounding_box().size.width as i32) / 2;
    Text::new(nav.ok, Point::new(mid - 12, y), nav_style)
        .draw(canvas)
        .ok();

    Text::new(nav.left, Point::new(mid - 70, y), nav_style)
        .draw(canvas)
        .ok();

    Text::new(nav.right, Point::new(mid + 40, y), nav_style)
        .draw(canvas)
        .ok();
}

pub fn draw_list_6<D>(
    canvas: &mut UiCanvas<'_, D>,
    l: &UiLayout,
    items: &[&str],
    selected: usize,
) where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    let pad = 16;
    let row_h = ((l.content.size.height as i32) - (pad * 2)) / 6;

    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    let text_style_inv = MonoTextStyle::new(&FONT_10X20, BinaryColor::Off);

    for row in 0..6 {
        let y = l.content.top_left.y + pad + (row * row_h);
        let r = Rectangle::new(
            Point::new(l.content.top_left.x + pad, y),
            Size::new(
                (l.content.size.width as i32 - (pad * 2)) as u32,
                (row_h - 10).max(20) as u32,
            ),
        );

        let is_sel = row as usize == selected;

        if is_sel {
            let style = PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .fill_color(BinaryColor::On)
                .build();
            r.into_styled(style).draw(canvas).ok();
        } else {
            r.into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(canvas)
                .ok();
        }

        if row < items.len() as i32 {
            let label = items[row as usize];
            let t_style = if is_sel { text_style_inv } else { text_style };
            Text::new(label, Point::new(r.top_left.x + 14, r.top_left.y + 26), t_style)
                .draw(canvas)
                .ok();
        }
    }
}

pub fn draw_grid_2x3<D>(
    canvas: &mut UiCanvas<'_, D>,
    l: &UiLayout,
    items: &[&str],
    selected: usize,
) where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    let pad = 16;
    let gap = 12;

    let avail_w = l.content.size.width as i32 - (pad * 2) - gap;
    let avail_h = l.content.size.height as i32 - (pad * 2) - (gap * 2);

    let card_w = avail_w / 2;
    let card_h = avail_h / 3;

    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    let text_style_inv = MonoTextStyle::new(&FONT_10X20, BinaryColor::Off);

    for idx in 0..6 {
        let col = idx % 2;
        let row = idx / 2;

        let x = l.content.top_left.x + pad + (col as i32 * (card_w + gap));
        let y = l.content.top_left.y + pad + (row as i32 * (card_h + gap));

        let r = Rectangle::new(Point::new(x, y), Size::new(card_w as u32, card_h as u32));
        let is_sel = idx == selected;

        if is_sel {
            let style = PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(2)
                .fill_color(BinaryColor::On)
                .build();
            r.into_styled(style).draw(canvas).ok();
        } else {
            r.into_styled(PrimitiveStyleBuilder::new().stroke_color(BinaryColor::On).stroke_width(1).build())
                .draw(canvas)
                .ok();
        }

        if idx < items.len() {
            let label = items[idx];
            let t_style = if is_sel { text_style_inv } else { text_style };
            let text_w = (label.len() as i32) * 10;
            let tx = x + (card_w - text_w) / 2;
            let ty = y + (card_h / 2) + 8;
            Text::new(label, Point::new(tx.max(x + 10), ty), t_style)
                .draw(canvas)
                .ok();
        }
    }
}

pub fn draw_reader_content<D>(canvas: &mut UiCanvas<'_, D>, layout: &UiLayout, content: &str, page: usize)
where
    D: embedded_hal_async::spi::SpiDevice<u8>,
{
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    
    let content_area = layout.content;
    let margin = 10;
    let line_height = 24;
    let chars_per_line = ((content_area.size.width as i32 - margin * 2) / 10) as usize;
    let lines_per_page = ((content_area.size.height as i32 - margin * 2) / line_height) as usize;
    
    if content.is_empty() {
        Text::new(
            "Loading...",
            Point::new(content_area.top_left.x + margin, content_area.top_left.y + margin + 20),
            text_style,
        )
        .draw(canvas)
        .ok();
        return;
    }
    
    // Simple word-wrap and pagination
    let mut y = content_area.top_left.y + margin + 20;
    let mut line_count = 0;
    let mut char_count = 0;
    let start_char = page * chars_per_line * lines_per_page;
    
    for line in content.lines() {
        if char_count + line.len() < start_char {
            char_count += line.len() + 1;
            continue;
        }
        
        // Draw this line (possibly truncated)
        let display_line = if line.len() > chars_per_line {
            &line[..chars_per_line]
        } else {
            line
        };
        
        Text::new(
            display_line,
            Point::new(content_area.top_left.x + margin, y),
            text_style,
        )
        .draw(canvas)
        .ok();
        
        y += line_height;
        line_count += 1;
        
        if line_count >= lines_per_page {
            break;
        }
    }
    
    // Draw page indicator at bottom
    let mut page_str: heapless::String<16> = heapless::String::new();
    let _ = core::fmt::write(&mut page_str, format_args!("Page {}", page + 1));
    
    let page_y = content_area.top_left.y + content_area.size.height as i32 - 5;
    let page_x = content_area.top_left.x + (content_area.size.width as i32 / 2) - 30;
    Text::new(page_str.as_str(), Point::new(page_x, page_y), text_style)
        .draw(canvas)
        .ok();
}
