//! The conversion between a raw number in the ECU's memory and the value a person reads ("0.375*X+12.0" turns a stored 0..255
//! into degrees of crank angle). Definition files write it as a formula in X; Icarus evaluates it to show a value and inverts
//! it to store one.
//!
//! The grammar: numbers (`12`, `0.375`, `1e-3`, `0x1F`), the variable `X`, `+ - * /`, `^` (power), unary minus and parentheses.

use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    Num(f64),
    X,
    Neg(Box<Expr>),
    Bin(Op, Box<Expr>, Box<Expr>),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Op {
    Add,
    Sub,
    Mul,
    Div,
    Pow,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ExprError(pub String);

impl fmt::Display for ExprError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ExprError {}

impl Expr {
    /// The identity, X: a value stored as it is read.
    pub fn identity() -> Expr {
        Expr::X
    }

    pub fn parse(text: &str) -> Result<Expr, ExprError> {
        let mut p = Parser {
            s: text.as_bytes(),
            i: 0,
        };
        let e = p.expr()?;
        p.ws();
        if p.i != p.s.len() {
            return Err(ExprError(format!(
                "unexpected '{}' at {} in \"{text}\"",
                p.s[p.i] as char, p.i
            )));
        }
        Ok(e)
    }

    pub fn eval(&self, x: f64) -> f64 {
        match self {
            Expr::Num(n) => *n,
            Expr::X => x,
            Expr::Neg(a) => -a.eval(x),
            Expr::Bin(op, a, b) => {
                let (a, b) = (a.eval(x), b.eval(x));
                match op {
                    Op::Add => a + b,
                    Op::Sub => a - b,
                    Op::Mul => a * b,
                    Op::Div => a / b,
                    Op::Pow => a.powf(b),
                }
            }
        }
    }

    /// (a, b) when the formula is a*X + b (almost all of them are), checked at several points.
    pub fn affine(&self) -> Option<(f64, f64)> {
        let b = self.eval(0.0);
        let a = self.eval(1.0) - b;
        if !a.is_finite() || !b.is_finite() || a == 0.0 {
            return None;
        }
        for x in [-1000.0, -3.0, 2.0, 7.5, 255.0, 65535.0, 1.0e6] {
            let want = a * x + b;
            let got = self.eval(x);
            if !got.is_finite() || (got - want).abs() > 1e-9 * want.abs().max(1.0) {
                return None;
            }
        }
        Some((a, b))
    }

    /// The raw value in `lo..=hi` whose converted value is closest to `value`: exact for a*X+b, otherwise a search (every raw
    /// value for ranges up to 2^20, else bisection, which assumes the formula keeps rising or falling).
    pub fn invert_int(&self, value: f64, lo: i64, hi: i64) -> Result<i64, ExprError> {
        if !value.is_finite() {
            return Err(ExprError("not a number".into()));
        }
        if let Some((a, b)) = self.affine() {
            let raw = ((value - b) / a).round();
            return Ok(raw.clamp(lo as f64, hi as f64) as i64);
        }
        let dist = |r: i64| (self.eval(r as f64) - value).abs();
        if hi - lo <= 1 << 20 {
            let mut best = (lo, f64::INFINITY);
            for r in lo..=hi {
                let d = dist(r);
                if d < best.1 {
                    best = (r, d);
                }
            }
            if best.1.is_finite() {
                return Ok(best.0);
            }
            return Err(ExprError(
                "the formula gives no number in the stored range".into(),
            ));
        }
        let rising = self.eval(hi as f64) >= self.eval(lo as f64);
        let (mut a, mut b) = (lo, hi);
        while b - a > 1 {
            let m = a + (b - a) / 2;
            if (self.eval(m as f64) < value) == rising {
                a = m;
            } else {
                b = m;
            }
        }
        Ok(if dist(a) <= dist(b) { a } else { b })
    }

    /// The raw value of a floating-point cell for `value` (a*X+b only).
    pub fn invert_float(&self, value: f64) -> Result<f64, ExprError> {
        match self.affine() {
            Some((a, b)) => Ok((value - b) / a),
            None => Err(ExprError(
                "only a*X+b formulas can be inverted for floating-point data".into(),
            )),
        }
    }
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
}

impl Parser<'_> {
    fn ws(&mut self) {
        while self.i < self.s.len() && self.s[self.i].is_ascii_whitespace() {
            self.i += 1;
        }
    }

    fn peek(&mut self) -> Option<u8> {
        self.ws();
        self.s.get(self.i).copied()
    }

    fn expr(&mut self) -> Result<Expr, ExprError> {
        let mut e = self.term()?;
        while let Some(c @ (b'+' | b'-')) = self.peek() {
            self.i += 1;
            let r = self.term()?;
            e = Expr::Bin(
                if c == b'+' { Op::Add } else { Op::Sub },
                Box::new(e),
                Box::new(r),
            );
        }
        Ok(e)
    }

    fn term(&mut self) -> Result<Expr, ExprError> {
        let mut e = self.unary()?;
        while let Some(c @ (b'*' | b'/')) = self.peek() {
            self.i += 1;
            let r = self.unary()?;
            e = Expr::Bin(
                if c == b'*' { Op::Mul } else { Op::Div },
                Box::new(e),
                Box::new(r),
            );
        }
        Ok(e)
    }

    fn unary(&mut self) -> Result<Expr, ExprError> {
        match self.peek() {
            Some(b'-') => {
                self.i += 1;
                Ok(Expr::Neg(Box::new(self.unary()?)))
            }
            Some(b'+') => {
                self.i += 1;
                self.unary()
            }
            _ => self.power(),
        }
    }

    fn power(&mut self) -> Result<Expr, ExprError> {
        let base = self.primary()?;
        if self.peek() == Some(b'^') {
            self.i += 1;
            let exp = self.unary()?; // right-associative: 2^3^2 = 2^9
            return Ok(Expr::Bin(Op::Pow, Box::new(base), Box::new(exp)));
        }
        Ok(base)
    }

    fn primary(&mut self) -> Result<Expr, ExprError> {
        match self.peek() {
            Some(b'(') => {
                self.i += 1;
                let e = self.expr()?;
                if self.peek() != Some(b')') {
                    return Err(ExprError("missing ')'".into()));
                }
                self.i += 1;
                Ok(e)
            }
            Some(b'X' | b'x') => {
                self.i += 1;
                if self
                    .s
                    .get(self.i)
                    .is_some_and(|c| c.is_ascii_alphanumeric() || *c == b'_')
                {
                    return Err(ExprError("only the variable X is supported".into()));
                }
                Ok(Expr::X)
            }
            Some(c) if c.is_ascii_digit() || c == b'.' => self.number(),
            Some(c) if c.is_ascii_alphabetic() => Err(ExprError(
                "only the variable X is supported (no functions or other variables)".into(),
            )),
            Some(c) => Err(ExprError(format!("unexpected '{}'", c as char))),
            None => Err(ExprError("the formula ends too early".into())),
        }
    }

    fn number(&mut self) -> Result<Expr, ExprError> {
        let start = self.i;
        if self.s[self.i..].starts_with(b"0x") || self.s[self.i..].starts_with(b"0X") {
            self.i += 2;
            let h = self.i;
            while self.i < self.s.len() && self.s[self.i].is_ascii_hexdigit() {
                self.i += 1;
            }
            let t = std::str::from_utf8(&self.s[h..self.i]).unwrap();
            return u64::from_str_radix(t, 16)
                .map(|n| Expr::Num(n as f64))
                .map_err(|_| ExprError("bad hex number".into()));
        }
        while self.i < self.s.len() && (self.s[self.i].is_ascii_digit() || self.s[self.i] == b'.') {
            self.i += 1;
        }
        if self.i < self.s.len() && (self.s[self.i] == b'e' || self.s[self.i] == b'E') {
            let save = self.i;
            self.i += 1;
            if self.i < self.s.len() && (self.s[self.i] == b'+' || self.s[self.i] == b'-') {
                self.i += 1;
            }
            if self.i < self.s.len() && self.s[self.i].is_ascii_digit() {
                while self.i < self.s.len() && self.s[self.i].is_ascii_digit() {
                    self.i += 1;
                }
            } else {
                self.i = save;
            }
        }
        let t = std::str::from_utf8(&self.s[start..self.i]).unwrap();
        t.parse::<f64>()
            .map(Expr::Num)
            .map_err(|_| ExprError(format!("bad number \"{t}\"")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(s: &str, x: f64) -> f64 {
        Expr::parse(s).unwrap().eval(x)
    }

    #[test]
    fn formulas_from_definition_files() {
        assert_eq!(ev("X", 7.0), 7.0);
        assert_eq!(ev("0.375*X+12.0", 8.0), 15.0);
        assert_eq!(ev("0.75*X-48.0", 64.0), 0.0);
        assert_eq!(ev("(X*100)/255", 255.0), 100.0);
        assert_eq!(ev("X/2-40", 100.0), 10.0);
        assert_eq!(ev("-X", 3.0), -3.0);
        assert_eq!(ev("2^3^2", 0.0), 512.0);
        assert_eq!(ev("1e-3*X", 1000.0), 1.0);
        assert_eq!(ev("0x10*X", 2.0), 32.0);
        assert_eq!(ev(" 1.0 * x ", 4.0), 4.0);
    }

    #[test]
    fn bad_formulas_are_errors_not_guesses() {
        for bad in ["", "X+", "(X", "Y*2", "X2", "sin(X)", "X $ 2", "1..2"] {
            assert!(Expr::parse(bad).is_err(), "{bad:?} should not parse");
        }
    }

    #[test]
    fn inverting_finds_the_stored_value() {
        let e = Expr::parse("0.375*X+12.0").unwrap();
        assert_eq!(e.affine(), Some((0.375, 12.0)));
        assert_eq!(e.invert_int(15.0, 0, 255).unwrap(), 8);
        assert_eq!(
            e.invert_int(15.1, 0, 255).unwrap(),
            8,
            "the nearest stored value"
        );
        assert_eq!(
            e.invert_int(1000.0, 0, 255).unwrap(),
            255,
            "kept inside the cell"
        );
        let sq = Expr::parse("X*X/100").unwrap();
        assert_eq!(sq.affine(), None);
        assert_eq!(sq.invert_int(25.0, 0, 255).unwrap(), 50);
        assert_eq!(
            Expr::parse("0.5*X").unwrap().invert_float(3.0).unwrap(),
            6.0
        );
        assert!(sq.invert_float(1.0).is_err());
        assert!(e.invert_int(f64::NAN, 0, 255).is_err());
    }
}
