use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::units::{DisplayUnit, MM_PER_INCH};

/// Errors produced while tokenizing or evaluating a parametric math/unit expression.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum ExprError {
    #[error("expression cannot be empty")]
    EmptyExpression,
    #[error("unexpected character '{ch}' at byte {pos}")]
    UnexpectedChar { ch: char, pos: usize },
    #[error("invalid number literal '{literal}'")]
    InvalidNumber { literal: String },
    #[error("unexpected token '{found}', expected {expected}")]
    UnexpectedToken {
        found: String,
        expected: &'static str,
    },
    #[error("unmatched closing parenthesis")]
    UnmatchedCloseParen,
    #[error("missing closing parenthesis ')'")]
    MissingCloseParen,
    #[error("unknown variable or unit '{name}'")]
    UnknownIdentifier { name: String },
    #[error("unknown function '{name}'")]
    UnknownFunction { name: String },
    #[error("function '{name}' expects {expected} argument(s), got {got}")]
    WrongArity {
        name: String,
        expected: usize,
        got: usize,
    },
    #[error("division by zero")]
    DivisionByZero,
    #[error("expression evaluated to non-finite value (NaN or Inf)")]
    NonFiniteResult,
}

/// Scoped variables available when evaluating operation or setup parameter formulas.
/// Length variables are stored in canonical `f64` millimeters; dimensionless/angle variables
/// (such as `flutes` or `tip_angle` in radians) are stored as scalars.
#[derive(Debug, Clone, Default)]
pub struct EvalContext {
    vars: BTreeMap<String, f64>,
    scalar_vars: BTreeMap<String, f64>,
    pub default_unit: DisplayUnit,
}

impl EvalContext {
    pub fn new(default_unit: DisplayUnit) -> Self {
        Self {
            vars: BTreeMap::new(),
            scalar_vars: BTreeMap::new(),
            default_unit,
        }
    }

    pub fn with_var(mut self, name: impl Into<String>, val_mm: f64) -> Self {
        self.vars.insert(name.into(), val_mm);
        self
    }

    pub fn set_var(&mut self, name: impl Into<String>, val_mm: f64) {
        self.vars.insert(name.into(), val_mm);
    }

    pub fn get_var(&self, name: &str) -> Option<f64> {
        self.vars.get(name).copied()
    }

    /// Inserts a dimensionless or radian-angle variable (e.g., `flutes`, `tip_angle`).
    pub fn with_scalar_var(mut self, name: impl Into<String>, val: f64) -> Self {
        self.scalar_vars.insert(name.into(), val);
        self
    }

    pub fn set_scalar_var(&mut self, name: impl Into<String>, val: f64) {
        self.scalar_vars.insert(name.into(), val);
    }

    pub fn get_scalar_var(&self, name: &str) -> Option<f64> {
        self.scalar_vars.get(name).copied()
    }
}

/// Persisted length parameter preserving an optional user formula alongside its canonical `f64` mm value.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParamLength {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expr: Option<String>,
    pub value_mm: f64,
}

impl ParamLength {
    pub fn from_mm(value_mm: f64) -> Self {
        Self {
            expr: None,
            value_mm,
        }
    }

    pub fn parse(input: &str, ctx: &EvalContext) -> Result<Self, ExprError> {
        let trimmed = input.trim();
        let value_mm = evaluate_length_mm(trimmed, ctx)?;
        let is_plain_number = trimmed.parse::<f64>().is_ok();
        let expr = if is_plain_number && ctx.default_unit == DisplayUnit::Millimeter {
            None
        } else {
            Some(trimmed.to_string())
        };
        Ok(Self { expr, value_mm })
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) -> Result<(), ExprError> {
        if let Some(expr) = &self.expr {
            self.value_mm = evaluate_length_mm(expr, ctx)?;
        }
        Ok(())
    }
}

/// Evaluates an inline arithmetic/unit expression into canonical `f64` millimeters.
///
/// Semantics:
/// - Explicit unit suffixes (`mm`, `cm`, `m`, `in`, `"`, `thou`, `mil`) scale the immediately
///   preceding term directly into millimeters, regardless of `ctx.default_unit`.
/// - Variables (`tool_d`, `stock_z`, etc.) are already stored in millimeters in `EvalContext`.
/// - Bare unitless numbers (`0.25` when `default_unit == Inch`) are scaled by `default_unit`
///   ONLY when the expression contains no explicit length units or millimeter variables, OR when
///   attached to a unit suffix (`1/4 in` parses cleanly as `(1 / 4) in` == `6.35 mm`!).
pub fn evaluate_length_mm(input: &str, ctx: &EvalContext) -> Result<f64, ExprError> {
    let trimmed = input.trim();
    if trimmed.is_empty() {
        return Err(ExprError::EmptyExpression);
    }
    let tokens = tokenize(trimmed)?;
    let mut parser = Parser {
        tokens: &tokens,
        pos: 0,
        ctx,
    };
    let val = parser.parse_expr()?;
    if parser.pos < parser.tokens.len() {
        let leftover = &parser.tokens[parser.pos];
        if matches!(leftover, Token::RParen) {
            return Err(ExprError::UnmatchedCloseParen);
        }
        return Err(ExprError::UnexpectedToken {
            found: leftover.describe(),
            expected: "end of expression",
        });
    }
    let mm = match val.dim {
        Dim::Millimeters => val.number,
        Dim::Scalar => ctx.default_unit.to_mm(val.number),
    };
    if !mm.is_finite() {
        return Err(ExprError::NonFiniteResult);
    }
    Ok(mm)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dim {
    /// Dimensionless scalar (will be converted via `ctx.default_unit` at root if still scalar).
    Scalar,
    /// Already converted into canonical millimeters.
    Millimeters,
}

#[derive(Debug, Clone, Copy)]
struct Val {
    number: f64,
    dim: Dim,
}

#[derive(Debug, Clone, PartialEq)]
enum Token {
    Number(f64),
    Ident(String),
    QuoteInch,
    Plus,
    Minus,
    Star,
    Slash,
    LParen,
    RParen,
    Comma,
}

impl Token {
    fn describe(&self) -> String {
        match self {
            Self::Number(v) => v.to_string(),
            Self::Ident(s) => s.clone(),
            Self::QuoteInch => "\"".to_string(),
            Self::Plus => "+".to_string(),
            Self::Minus => "-".to_string(),
            Self::Star => "*".to_string(),
            Self::Slash => "/".to_string(),
            Self::LParen => "(".to_string(),
            Self::RParen => ")".to_string(),
            Self::Comma => ",".to_string(),
        }
    }
}

fn unit_scale_to_mm(ident: &str) -> Option<f64> {
    match ident {
        "mm" | "millimeter" | "millimeters" => Some(1.0),
        "cm" | "centimeter" | "centimeters" => Some(10.0),
        "m" | "meter" | "meters" => Some(1000.0),
        "in" | "inch" | "inches" => Some(MM_PER_INCH),
        "thou" | "mil" | "mils" => Some(0.0254),
        "um" | "micron" | "microns" => Some(0.001),
        _ => None,
    }
}

fn angle_scale_to_rad(ident: &str) -> Option<f64> {
    match ident {
        "deg" | "degree" | "degrees" => Some(std::f64::consts::PI / 180.0),
        "rad" | "radian" | "radians" => Some(1.0),
        _ => None,
    }
}

fn tokenize(s: &str) -> Result<Vec<Token>, ExprError> {
    let mut tokens = Vec::new();
    let bytes = s.as_bytes();
    let mut i = 0;

    while i < bytes.len() {
        let ch = s[i..].chars().next().unwrap_or('\0');
        if ch.is_whitespace() {
            i += ch.len_utf8();
            continue;
        }
        match ch {
            '+' => {
                tokens.push(Token::Plus);
                i += 1;
            }
            '-' => {
                tokens.push(Token::Minus);
                i += 1;
            }
            '*' => {
                tokens.push(Token::Star);
                i += 1;
            }
            '/' => {
                tokens.push(Token::Slash);
                i += 1;
            }
            '(' => {
                tokens.push(Token::LParen);
                i += 1;
            }
            ')' => {
                tokens.push(Token::RParen);
                i += 1;
            }
            ',' => {
                tokens.push(Token::Comma);
                i += 1;
            }
            '"' => {
                tokens.push(Token::QuoteInch);
                i += 1;
            }
            c if c.is_ascii_digit() || c == '.' => {
                let start = i;
                let mut saw_dot = c == '.';
                i += 1;
                while i < bytes.len() {
                    let b = bytes[i];
                    if b.is_ascii_digit() {
                        i += 1;
                    } else if b == b'.' && !saw_dot {
                        saw_dot = true;
                        i += 1;
                    } else if (b == b'e' || b == b'E') && i + 1 < bytes.len() {
                        let next = bytes[i + 1];
                        if next.is_ascii_digit()
                            || ((next == b'+' || next == b'-')
                                && i + 2 < bytes.len()
                                && bytes[i + 2].is_ascii_digit())
                        {
                            i += 2;
                            while i < bytes.len() && bytes[i].is_ascii_digit() {
                                i += 1;
                            }
                        } else {
                            break;
                        }
                    } else {
                        break;
                    }
                }
                let literal = &s[start..i];
                let num = literal
                    .parse::<f64>()
                    .map_err(|_| ExprError::InvalidNumber {
                        literal: literal.to_string(),
                    })?;
                tokens.push(Token::Number(num));
            }
            c if c.is_ascii_alphabetic() || c == '_' => {
                let start = i;
                i += 1;
                while i < bytes.len() {
                    let b = bytes[i];
                    if b.is_ascii_alphanumeric() || b == b'_' {
                        i += 1;
                    } else {
                        break;
                    }
                }
                tokens.push(Token::Ident(s[start..i].to_string()));
            }
            other => {
                return Err(ExprError::UnexpectedChar { ch: other, pos: i });
            }
        }
    }

    Ok(tokens)
}

struct Parser<'a> {
    tokens: &'a [Token],
    pos: usize,
    ctx: &'a EvalContext,
}

impl<'a> Parser<'a> {
    fn peek(&self) -> Option<&'a Token> {
        self.tokens.get(self.pos)
    }

    /// `expr := add_sub`
    fn parse_expr(&mut self) -> Result<Val, ExprError> {
        let mut lhs = self.parse_mul_div()?;
        while let Some(tok) = self.peek() {
            let is_add = match tok {
                Token::Plus => true,
                Token::Minus => false,
                _ => break,
            };
            self.pos += 1;
            let rhs = self.parse_mul_div()?;
            lhs = self.combine_additive(lhs, rhs, is_add);
        }
        Ok(lhs)
    }

    fn combine_additive(&self, lhs: Val, rhs: Val, is_add: bool) -> Val {
        let (l_num, r_num, dim) = match (lhs.dim, rhs.dim) {
            (Dim::Scalar, Dim::Scalar) => (lhs.number, rhs.number, Dim::Scalar),
            (Dim::Millimeters, Dim::Millimeters) => (lhs.number, rhs.number, Dim::Millimeters),
            (Dim::Millimeters, Dim::Scalar) => (
                lhs.number,
                self.ctx.default_unit.to_mm(rhs.number),
                Dim::Millimeters,
            ),
            (Dim::Scalar, Dim::Millimeters) => (
                self.ctx.default_unit.to_mm(lhs.number),
                rhs.number,
                Dim::Millimeters,
            ),
        };
        Val {
            number: if is_add { l_num + r_num } else { l_num - r_num },
            dim,
        }
    }

    /// `mul_div := unary (('*' | '/') unary)* [trailing_unit_on_fraction]`
    /// Supporting `1/4 in` naturally (`(1 / 4)` followed by `in` == `0.25 in` == `6.35 mm`)!
    fn parse_mul_div(&mut self) -> Result<Val, ExprError> {
        let mut lhs = self.parse_unary()?;
        while let Some(tok) = self.peek() {
            match tok {
                Token::Star => {
                    self.pos += 1;
                    let rhs = self.parse_unary()?;
                    let dim = if lhs.dim == Dim::Millimeters || rhs.dim == Dim::Millimeters {
                        Dim::Millimeters
                    } else {
                        Dim::Scalar
                    };
                    lhs = Val {
                        number: lhs.number * rhs.number,
                        dim,
                    };
                }
                Token::Slash => {
                    self.pos += 1;
                    let rhs = self.parse_unary()?;
                    if rhs.number.abs() < f64::EPSILON * 1e-3 {
                        return Err(ExprError::DivisionByZero);
                    }
                    // If dividing `1 / 4 in`, `rhs` would only carry `in` if `parse_unary` ate it;
                    // Instead, `parse_postfix_unit` attaches units tightly, EXCEPT when `lhs` is
                    // a pure fraction like `1/4 in`! Check carefully: if `1 / 4` has `4 in` on rhs,
                    // in CNC machinist notation `1/4 in` ALWAYS means `(1/4) in`, never `1 / (4 in)`!
                    let dim = match (lhs.dim, rhs.dim) {
                        (Dim::Millimeters, Dim::Millimeters) => Dim::Scalar,
                        (Dim::Millimeters, Dim::Scalar) => Dim::Millimeters,
                        (Dim::Scalar, Dim::Scalar) => Dim::Scalar,
                        // Machinist fraction rule: `<scalar> / <number><unit>` (e.g. `1/4 in` or `3/16"`):
                        (Dim::Scalar, Dim::Millimeters) => Dim::Millimeters,
                    };
                    let number = if lhs.dim == Dim::Scalar && rhs.dim == Dim::Millimeters {
                        // Note: only happens if rhs came from a literal with unit suffix or mm var;
                        // handled cleanly below by deferring unit suffix past `/` on numeric fractions!
                        lhs.number / rhs.number
                    } else {
                        lhs.number / rhs.number
                    };
                    lhs = Val { number, dim };
                }
                _ => break,
            }
            // Check for trailing unit suffix after a product/fraction such as `1/4 in` or `(2 + 3) mm`
            lhs = self.consume_unit_suffix(lhs);
        }
        Ok(lhs)
    }

    fn parse_unary(&mut self) -> Result<Val, ExprError> {
        match self.peek() {
            Some(Token::Plus) => {
                self.pos += 1;
                self.parse_unary()
            }
            Some(Token::Minus) => {
                self.pos += 1;
                let mut inner = self.parse_unary()?;
                inner.number = -inner.number;
                Ok(inner)
            }
            _ => self.parse_primary(),
        }
    }

    fn parse_primary(&mut self) -> Result<Val, ExprError> {
        let tok = self.peek().cloned().ok_or(ExprError::UnexpectedToken {
            found: "end of input".to_string(),
            expected: "number, variable, or '('",
        })?;

        let base = match tok {
            Token::Number(n) => {
                self.pos += 1;
                // Lookahead: if this number is immediately followed by `/` (like `1/4 in`), do NOT
                // consume a unit on the denominator inside `parse_primary`—let `parse_mul_div` apply
                // the unit suffix to the completed fraction `1/4`!
                Val {
                    number: n,
                    dim: Dim::Scalar,
                }
            }
            Token::LParen => {
                self.pos += 1;
                let inner = self.parse_expr()?;
                match self.peek() {
                    Some(Token::RParen) => {
                        self.pos += 1;
                        inner
                    }
                    _ => return Err(ExprError::MissingCloseParen),
                }
            }
            Token::Ident(name) => {
                self.pos += 1;
                if matches!(self.peek(), Some(Token::LParen)) {
                    self.pos += 1;
                    let mut args = Vec::new();
                    if !matches!(self.peek(), Some(Token::RParen)) {
                        loop {
                            args.push(self.parse_expr()?);
                            if matches!(self.peek(), Some(Token::Comma)) {
                                self.pos += 1;
                            } else {
                                break;
                            }
                        }
                    }
                    if !matches!(self.peek(), Some(Token::RParen)) {
                        return Err(ExprError::MissingCloseParen);
                    }
                    self.pos += 1;
                    self.eval_function(&name, &args)?
                } else if name.eq_ignore_ascii_case("pi") {
                    Val {
                        number: std::f64::consts::PI,
                        dim: Dim::Scalar,
                    }
                } else if let Some(var_mm) = self.ctx.get_var(&name) {
                    Val {
                        number: var_mm,
                        dim: Dim::Millimeters,
                    }
                } else if let Some(scalar) = self.ctx.get_scalar_var(&name) {
                    Val {
                        number: scalar,
                        dim: Dim::Scalar,
                    }
                } else {
                    return Err(ExprError::UnknownIdentifier { name });
                }
            }
            other => {
                return Err(ExprError::UnexpectedToken {
                    found: other.describe(),
                    expected: "number, variable, or '('",
                });
            }
        };

        // If the previous token is `/` (we are parsing the denominator of `a / b in`),
        // leave any trailing unit suffix for `parse_mul_div` so `1 / 4 in` == `(1 / 4) * 25.4`!
        let prev_was_slash =
            self.pos >= 2 && matches!(self.tokens.get(self.pos - 2), Some(Token::Slash));
        if prev_was_slash {
            Ok(base)
        } else {
            Ok(self.consume_unit_suffix(base))
        }
    }

    fn consume_unit_suffix(&mut self, val: Val) -> Val {
        if val.dim == Dim::Millimeters {
            return val;
        }
        match self.peek() {
            Some(Token::QuoteInch) => {
                self.pos += 1;
                Val {
                    number: val.number * MM_PER_INCH,
                    dim: Dim::Millimeters,
                }
            }
            Some(Token::Ident(id)) => {
                let lower = id.to_ascii_lowercase();
                if let Some(scale) = unit_scale_to_mm(&lower) {
                    self.pos += 1;
                    Val {
                        number: val.number * scale,
                        dim: Dim::Millimeters,
                    }
                } else if let Some(rad_scale) = angle_scale_to_rad(&lower) {
                    self.pos += 1;
                    Val {
                        number: val.number * rad_scale,
                        dim: Dim::Scalar,
                    }
                } else {
                    val
                }
            }
            _ => val,
        }
    }

    fn eval_function(&self, name: &str, args: &[Val]) -> Result<Val, ExprError> {
        let lower = name.to_ascii_lowercase();
        match lower.as_str() {
            "abs" | "sqrt" | "sin" | "cos" | "tan" => {
                if args.len() != 1 {
                    return Err(ExprError::WrongArity {
                        name: name.to_string(),
                        expected: 1,
                        got: args.len(),
                    });
                }
                let a = args[0];
                let (number, dim) = match lower.as_str() {
                    "abs" => (a.number.abs(), a.dim),
                    "sqrt" => (a.number.sqrt(), a.dim),
                    "sin" => (a.number.sin(), Dim::Scalar),
                    "cos" => (a.number.cos(), Dim::Scalar),
                    "tan" => (a.number.tan(), Dim::Scalar),
                    _ => unreachable!(),
                };
                Ok(Val { number, dim })
            }
            "min" | "max" => {
                if args.len() != 2 {
                    return Err(ExprError::WrongArity {
                        name: name.to_string(),
                        expected: 2,
                        got: args.len(),
                    });
                }
                let combined = self.combine_additive(
                    args[0],
                    Val {
                        number: 0.0,
                        dim: args[1].dim,
                    },
                    true,
                );
                let rhs_conv = self.combine_additive(
                    args[1],
                    Val {
                        number: 0.0,
                        dim: args[0].dim,
                    },
                    true,
                );
                let number = if lower == "min" {
                    combined.number.min(rhs_conv.number)
                } else {
                    combined.number.max(rhs_conv.number)
                };
                Ok(Val {
                    number,
                    dim: combined.dim,
                })
            }
            _ => Err(ExprError::UnknownFunction {
                name: name.to_string(),
            }),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::units::approx_eq;

    #[test]
    fn evaluates_machinist_fractions_and_mixed_units() {
        let ctx = EvalContext::new(DisplayUnit::Millimeter)
            .with_var("tool_d", 6.35)
            .with_var("stock_z", 15.0);

        assert!(approx_eq(
            evaluate_length_mm("1/4 in", &ctx).unwrap(),
            6.35,
            1e-9
        ));
        assert!(approx_eq(
            evaluate_length_mm("3/8\"", &ctx).unwrap(),
            9.525,
            1e-9
        ));
        assert!(approx_eq(
            evaluate_length_mm("tool_d * 0.4 + 10 thou", &ctx).unwrap(),
            6.35 * 0.4 + 0.254,
            1e-9
        ));
        assert!(approx_eq(
            evaluate_length_mm("stock_z - 0.5mm", &ctx).unwrap(),
            14.5,
            1e-9
        ));
        assert!(approx_eq(
            evaluate_length_mm("10mm * tan(45 deg)", &ctx).unwrap(),
            10.0,
            1e-9
        ));
    }

    #[test]
    fn respects_default_display_unit_for_bare_scalars() {
        let inch_ctx = EvalContext::new(DisplayUnit::Inch);
        assert!(approx_eq(
            evaluate_length_mm("0.5", &inch_ctx).unwrap(),
            12.7,
            1e-9
        ));
        assert!(approx_eq(
            evaluate_length_mm("6 mm", &inch_ctx).unwrap(),
            6.0,
            1e-9
        ));
    }

    #[test]
    fn reports_clean_errors_without_panicking() {
        let ctx = EvalContext::default();
        assert_eq!(
            evaluate_length_mm("   ", &ctx),
            Err(ExprError::EmptyExpression)
        );
        assert_eq!(
            evaluate_length_mm("10 / 0", &ctx),
            Err(ExprError::DivisionByZero)
        );
        assert!(matches!(
            evaluate_length_mm("unknown_var + 1", &ctx),
            Err(ExprError::UnknownIdentifier { .. })
        ));
        assert_eq!(
            evaluate_length_mm("(1 + 2", &ctx),
            Err(ExprError::MissingCloseParen)
        );
    }
}
