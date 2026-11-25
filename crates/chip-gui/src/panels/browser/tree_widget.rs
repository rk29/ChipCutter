use egui::{
    Color32, CornerRadius, CursorIcon, FontId, Id, Key, Layout, Modifiers, Pos2, Rect, Response,
    Sense, Shape, Stroke, StrokeKind, Ui, UiBuilder, Vec2,
    text::{LayoutJob, TextFormat, TextWrapping},
};

use crate::{
    theme::ThemeTokens,
    widgets::{IconKind, paint_icon},
};

/// Standard compact row height for the hierarchical Project Tree View.
pub const TREE_ROW_HEIGHT: f32 = 24.0;
/// Horizontal indentation step per hierarchy nesting level.
pub const TREE_INDENT_STEP: f32 = 11.0;
/// Size of procedural vector node icons inside tree rows.
pub const TREE_ICON_SIZE: f32 = 14.0;

pub(super) const TREE_LEFT_PAD: f32 = 3.0;
pub(super) const TREE_CHEVRON_W: f32 = 13.0;

/// Stable semantic identifier for every addressable row in the Project Tree.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum TreeNodeKey {
    Project,
    ModelsFolder,
    Model(usize),
    SketchesFolder,
    Sketch(usize),
    SketchEntity {
        sketch_idx: usize,
        entity_idx: usize,
    },
    AvoidRegionsFolder,
    AvoidRegion(usize),
    ToolsFolder,
    Tool(usize),
    SetupsFolder,
    Setup(usize),
    FixturesFolder {
        setup_idx: usize,
    },
    Fixture {
        setup_idx: usize,
        fixture_idx: usize,
    },
    OperationsFolder {
        setup_idx: usize,
    },
    Operation {
        setup_idx: usize,
        op_idx: usize,
    },
}

/// Entry recorded in visual order during tree rendering for keyboard navigation.
#[derive(Debug, Clone)]
pub struct VisibleTreeNode {
    pub key: TreeNodeKey,
    pub node_id: Id,
    pub parent: Option<TreeNodeKey>,
    pub has_children: bool,
    pub is_open: bool,
}

/// Logical keyboard navigation command consumed by the Project Tree.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TreeNavKey {
    Up,
    Down,
    Left,
    Right,
    Rename,
    Delete,
}

/// Concrete state transition emitted by [`resolve_tree_keyboard_nav`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TreeNavAction {
    Select(TreeNodeKey),
    SetOpen { node_id: Id, open: bool },
    Rename(TreeNodeKey),
    Delete(TreeNodeKey),
}

/// Resolves a single keyboard navigation key against the current visible tree row list.
pub fn resolve_tree_keyboard_nav(
    visible: &[VisibleTreeNode],
    focused: Option<&TreeNodeKey>,
    key: TreeNavKey,
) -> Vec<TreeNavAction> {
    if visible.is_empty() {
        return Vec::new();
    }
    let cur_idx = focused.and_then(|fk| visible.iter().position(|n| &n.key == fk));
    match key {
        TreeNavKey::Down => {
            let next_idx = match cur_idx {
                Some(i) => (i + 1).min(visible.len() - 1),
                None => 0,
            };
            vec![TreeNavAction::Select(visible[next_idx].key.clone())]
        }
        TreeNavKey::Up => {
            let prev_idx = match cur_idx {
                Some(i) => i.saturating_sub(1),
                None => visible.len() - 1,
            };
            vec![TreeNavAction::Select(visible[prev_idx].key.clone())]
        }
        TreeNavKey::Right => {
            let Some(i) = cur_idx else {
                return vec![TreeNavAction::Select(visible[0].key.clone())];
            };
            let node = &visible[i];
            if node.has_children && !node.is_open {
                vec![TreeNavAction::SetOpen {
                    node_id: node.node_id,
                    open: true,
                }]
            } else if node.has_children
                && node.is_open
                && let Some(child) = visible.get(i + 1)
                && child.parent.as_ref() == Some(&node.key)
            {
                vec![TreeNavAction::Select(child.key.clone())]
            } else {
                Vec::new()
            }
        }
        TreeNavKey::Left => {
            let Some(i) = cur_idx else {
                return Vec::new();
            };
            let node = &visible[i];
            if node.has_children && node.is_open {
                vec![TreeNavAction::SetOpen {
                    node_id: node.node_id,
                    open: false,
                }]
            } else if let Some(parent) = &node.parent {
                vec![TreeNavAction::Select(parent.clone())]
            } else {
                Vec::new()
            }
        }
        TreeNavKey::Rename => cur_idx
            .map(|i| vec![TreeNavAction::Rename(visible[i].key.clone())])
            .unwrap_or_default(),
        TreeNavKey::Delete => cur_idx
            .map(|i| vec![TreeNavAction::Delete(visible[i].key.clone())])
            .unwrap_or_default(),
    }
}

/// Polls and consumes a single tree-navigation key from `ui` when no text input has focus.
pub fn poll_tree_nav_key(ui: &mut Ui) -> Option<TreeNavKey> {
    if ui.ctx().egui_wants_keyboard_input() {
        return None;
    }
    ui.input_mut(|i| {
        if i.modifiers.command || i.modifiers.ctrl || i.modifiers.alt {
            return None;
        }
        if i.consume_key(Modifiers::NONE, Key::ArrowDown) {
            Some(TreeNavKey::Down)
        } else if i.consume_key(Modifiers::NONE, Key::ArrowUp) {
            Some(TreeNavKey::Up)
        } else if i.consume_key(Modifiers::NONE, Key::ArrowRight) {
            Some(TreeNavKey::Right)
        } else if i.consume_key(Modifiers::NONE, Key::ArrowLeft) {
            Some(TreeNavKey::Left)
        } else if i.consume_key(Modifiers::NONE, Key::F2) {
            Some(TreeNavKey::Rename)
        } else if i.consume_key(Modifiers::NONE, Key::Delete) {
            Some(TreeNavKey::Delete)
        } else {
            None
        }
    })
}

/// Returns the persisted expand/collapse state for `node_id`, falling back to `default_open`.
pub fn is_node_open(ui: &Ui, node_id: Id, default_open: bool) -> bool {
    ui.ctx()
        .data_mut(|d| d.get_persisted::<bool>(node_id))
        .unwrap_or(default_open)
}

/// Persists expand/collapse state for `node_id` in `egui` memory.
pub fn set_node_open(ui: &Ui, node_id: Id, open: bool) {
    ui.ctx().data_mut(|d| d.insert_persisted(node_id, open));
}

/// Configuration for a single row rendered via [`tree_row`].
pub struct TreeRowSpec<'a> {
    pub node_id: Id,
    pub depth: usize,
    pub icon: IconKind,
    pub icon_color: Color32,
    pub label: &'a str,
    pub label_color: Color32,
    pub bold: bool,
    pub strikethrough: bool,
    pub italic: bool,
    pub has_children: bool,
    pub default_open: bool,
    pub selected: bool,
    pub keyboard_focused: bool,
    pub draggable: bool,
    pub tooltip: Option<&'a str>,
}

/// Interactive result returned by [`tree_row`].
pub struct TreeRowOutput {
    pub row_rect: Rect,
    pub response: Response,
    pub is_open: bool,
    pub chevron_toggled: bool,
}

/// Renders a single lightweight, theme-aware Project Tree row with vertical guide lines,
/// persisted procedural chevron disclosure, `14 px` procedural vector icon, single-line
/// truncated label, and dynamically measured right-aligned badges/controls.
pub fn tree_row(
    ui: &mut Ui,
    spec: TreeRowSpec<'_>,
    add_trailing: impl FnOnce(&mut Ui, bool),
) -> TreeRowOutput {
    let tok = ThemeTokens::from_ui(ui);
    let mut is_open = if spec.has_children {
        is_node_open(ui, spec.node_id, spec.default_open)
    } else {
        false
    };

    let full_w = ui.available_width().max(120.0);
    let (_, row_rect) = ui.allocate_space(Vec2::new(full_w, TREE_ROW_HEIGHT));
    let bg_idx = ui.painter().add(Shape::Noop);

    // Subtle theme-matched vertical tree guide lines for ancestor nesting levels
    if ui.is_rect_visible(row_rect) {
        let guide_stroke = Stroke::new(1.0, tok.border.gamma_multiply(0.55));
        for lvl in 0..spec.depth {
            let gx = row_rect.left()
                + TREE_LEFT_PAD
                + (lvl as f32) * TREE_INDENT_STEP
                + TREE_CHEVRON_W * 0.5;
            ui.painter().line_segment(
                [
                    Pos2::new(gx, row_rect.top()),
                    Pos2::new(gx, row_rect.bottom()),
                ],
                guide_stroke,
            );
        }
    }

    let chev_x = row_rect.left() + TREE_LEFT_PAD + (spec.depth as f32) * TREE_INDENT_STEP;
    let main_left = chev_x + TREE_CHEVRON_W;
    let row_hovered_preview = ui.rect_contains_pointer(row_rect);

    // Right-aligned trailing badges & controls measured cleanly inside the row rect
    let trailing_bounds = Rect::from_min_max(
        Pos2::new(
            (main_left + 32.0).min(row_rect.right() - 4.0),
            row_rect.top(),
        ),
        Pos2::new(row_rect.right() - 4.0, row_rect.bottom()),
    );
    let mut trailing_ui = ui.new_child(
        UiBuilder::new()
            .max_rect(trailing_bounds)
            .layout(Layout::right_to_left(egui::Align::Center)),
    );
    trailing_ui.spacing_mut().item_spacing.x = 2.0;
    add_trailing(&mut trailing_ui, row_hovered_preview);
    let trailing_min = trailing_ui.min_rect();
    let main_right = if trailing_min.width() > 0.5 {
        (trailing_min.left() - 3.0).max(main_left + 24.0)
    } else {
        (row_rect.right() - 4.0).max(main_left + 24.0)
    };

    // Expand/collapse procedural chevron toggle
    let mut chevron_toggled = false;
    let mut chev_hovered = false;
    if spec.has_children {
        let chev_rect = Rect::from_min_size(
            Pos2::new(chev_x, row_rect.top()),
            Vec2::new(TREE_CHEVRON_W, TREE_ROW_HEIGHT),
        );
        let chev_resp = ui
            .interact(chev_rect, spec.node_id.with("chev"), Sense::click())
            .on_hover_cursor(CursorIcon::PointingHand);
        chev_hovered = chev_resp.hovered();
        if chev_resp.clicked() {
            is_open = !is_open;
            set_node_open(ui, spec.node_id, is_open);
            chevron_toggled = true;
        }
        if ui.is_rect_visible(chev_rect) {
            let chev_col = if chev_hovered {
                tok.accent
            } else {
                tok.text_muted
            };
            let glyph_rect = Rect::from_center_size(chev_rect.center(), Vec2::splat(10.0));
            paint_icon(
                ui.painter(),
                glyph_rect,
                if is_open {
                    IconKind::ChevronDown
                } else {
                    IconKind::ChevronRight
                },
                chev_col,
            );
        }
        let _ = chev_resp.on_hover_text(if is_open { "Collapse" } else { "Expand" });
    }

    // Main selectable/draggable content hit zone (icon + single-line truncated label)
    let main_rect = Rect::from_min_max(
        Pos2::new(main_left, row_rect.top()),
        Pos2::new(main_right, row_rect.bottom()),
    );
    let (sense, hover_cursor) = if spec.draggable {
        (Sense::click_and_drag(), CursorIcon::Grab)
    } else {
        (Sense::click(), CursorIcon::PointingHand)
    };
    let mut main_resp = ui
        .interact(main_rect, spec.node_id.with("row"), sense)
        .on_hover_cursor(hover_cursor);
    if let Some(tip) = spec.tooltip {
        main_resp = main_resp.on_hover_text(tip);
    }

    let hovered = row_hovered_preview || main_resp.hovered() || chev_hovered;

    if ui.is_rect_visible(row_rect) {
        // Paint row selection/hover/focus background behind all row content
        let mut bg_shapes = Vec::new();
        if spec.selected {
            bg_shapes.push(Shape::rect_filled(
                row_rect,
                CornerRadius::same(4),
                tok.accent_soft,
            ));
            let bar = Rect::from_min_max(
                Pos2::new(row_rect.left(), row_rect.top() + 2.0),
                Pos2::new(row_rect.left() + 2.5, row_rect.bottom() - 2.0),
            );
            bg_shapes.push(Shape::rect_filled(bar, CornerRadius::same(1), tok.accent));
        } else if hovered {
            bg_shapes.push(Shape::rect_filled(
                row_rect,
                CornerRadius::same(4),
                ui.visuals().widgets.hovered.bg_fill.gamma_multiply(0.65),
            ));
        }
        if spec.keyboard_focused {
            bg_shapes.push(Shape::rect_stroke(
                row_rect,
                CornerRadius::same(4),
                Stroke::new(1.0, tok.accent.gamma_multiply(0.75)),
                StrokeKind::Inside,
            ));
        }
        if !bg_shapes.is_empty() {
            ui.painter().set(bg_idx, Shape::Vec(bg_shapes));
        }

        // Paint crisp 14px procedural vector node icon
        let icon_rect = Rect::from_min_size(
            Pos2::new(
                main_rect.left() + 2.0,
                main_rect.center().y - TREE_ICON_SIZE * 0.5,
            ),
            Vec2::splat(TREE_ICON_SIZE),
        );
        paint_icon(ui.painter(), icon_rect, spec.icon, spec.icon_color);

        // Paint single-line truncated label
        let text_x = icon_rect.right() + 4.0;
        let text_max_w = (main_rect.right() - text_x).max(16.0);
        let font_id = FontId::proportional(if spec.bold { 12.5 } else { 12.0 });
        let mut fmt = TextFormat::simple(font_id, spec.label_color);
        fmt.italics = spec.italic;
        if spec.strikethrough {
            fmt.strikethrough = Stroke::new(1.0, spec.label_color);
        }
        let mut job = LayoutJob::single_section(spec.label.to_owned(), fmt);
        job.wrap = TextWrapping::truncate_at_width(text_max_w);
        let galley = ui.painter().layout_job(job);
        let text_pos = Pos2::new(text_x, main_rect.center().y - galley.size().y * 0.5);
        ui.painter().galley(text_pos, galley, spec.label_color);
    }

    TreeRowOutput {
        row_rect,
        response: main_resp,
        is_open,
        chevron_toggled,
    }
}

/// Renders an indented non-interactive muted hint row inside an expanded empty folder.
pub fn tree_hint_row(ui: &mut Ui, depth: usize, text: &str) {
    let tok = ThemeTokens::from_ui(ui);
    let full_w = ui.available_width().max(120.0);
    let (_, row_rect) = ui.allocate_space(Vec2::new(full_w, 20.0));
    if !ui.is_rect_visible(row_rect) {
        return;
    }
    let guide_stroke = Stroke::new(1.0, tok.border.gamma_multiply(0.55));
    for lvl in 0..depth {
        let gx = row_rect.left()
            + TREE_LEFT_PAD
            + (lvl as f32) * TREE_INDENT_STEP
            + TREE_CHEVRON_W * 0.5;
        ui.painter().line_segment(
            [
                Pos2::new(gx, row_rect.top()),
                Pos2::new(gx, row_rect.bottom()),
            ],
            guide_stroke,
        );
    }
    let text_x =
        row_rect.left() + TREE_LEFT_PAD + (depth as f32) * TREE_INDENT_STEP + TREE_CHEVRON_W + 2.0;
    let text_max_w = (row_rect.right() - text_x - 4.0).max(24.0);
    let mut job = LayoutJob::single_section(
        text.to_owned(),
        TextFormat::simple(FontId::proportional(11.0), tok.text_muted),
    );
    job.wrap = TextWrapping::truncate_at_width(text_max_w);
    let galley = ui.painter().layout_job(job);
    ui.painter().galley(
        Pos2::new(text_x, row_rect.center().y - galley.size().y * 0.5),
        galley,
        tok.text_muted,
    );
}

/// Renders a compact right-aligned pill badge (`G54`, `Active`, `T1 · 42s`, `3 ops`, etc.).
pub fn tree_badge_pill(
    ui: &mut Ui,
    text: &str,
    bg: Color32,
    border: Option<Color32>,
    fg: Color32,
    tooltip: Option<&str>,
) -> Response {
    let galley = ui
        .painter()
        .layout_no_wrap(text.to_owned(), FontId::proportional(10.5), fg);
    let pill_w = galley.size().x + 7.0;
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(pill_w, 16.0), Sense::hover());
    if ui.is_rect_visible(rect) {
        let stroke = border.map_or(Stroke::NONE, |b| Stroke::new(1.0, b));
        ui.painter()
            .rect(rect, CornerRadius::same(4), bg, stroke, StrokeKind::Inside);
        let text_pos = Pos2::new(
            rect.center().x - galley.size().x * 0.5,
            rect.center().y - galley.size().y * 0.5,
        );
        ui.painter().galley(text_pos, galley, fg);
    }
    if let Some(tip) = tooltip {
        resp.on_hover_text(tip)
    } else {
        resp
    }
}

/// Renders a compact `18×18` clickable `+` menu button on tree folder rows.
pub fn tree_folder_menu_button(
    ui: &mut Ui,
    id_salt: impl std::hash::Hash + std::fmt::Debug,
    tooltip: &str,
    add_menu: impl FnOnce(&mut Ui),
) {
    let tok = ThemeTokens::from_ui(ui);
    let (rect, resp) = ui.allocate_exact_size(Vec2::splat(18.0), Sense::click());
    if ui.is_rect_visible(rect) {
        let hovered = resp.hovered();
        let fg = if hovered {
            tok.text_primary
        } else {
            tok.text_muted
        };
        if hovered {
            ui.painter().rect(
                rect,
                CornerRadius::same(4),
                ui.visuals().widgets.hovered.bg_fill,
                Stroke::new(1.0, tok.accent),
                StrokeKind::Inside,
            );
        }
        paint_icon(
            ui.painter(),
            Rect::from_center_size(rect.center(), Vec2::splat(11.0)),
            IconKind::Plus,
            fg,
        );
    }
    let popup_id = ui.make_persistent_id(id_salt);
    egui::Popup::menu(&resp)
        .id(popup_id)
        .show(|ui| add_menu(ui));
    let _ = resp.on_hover_text(tooltip);
}
