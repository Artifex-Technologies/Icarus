//! NHTSA's public vehicle data (US Department of Transportation): the model, engine and drive a VIN stands for (vPIC), and
//! the recalls on record for a make, model and year. These go online: decoding a VIN sends that VIN to NHTSA, so it is
//! only done when the person asks; the recall list needs only make, model and year. Fetched with curl (HTTPS only).

use serde_json::Value;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct VinDecode {
    pub make: String,
    pub model: String,
    pub year: String,
    pub trim: String,
    pub engine: String,
    pub fuel: String,
    pub drive: String,
    pub body: String,
    pub plant_country: String,
    /// NHTSA's own remark when the VIN did not decode cleanly.
    pub note: Option<String>,
}

impl VinDecode {
    /// "2017 Toyota Tacoma 3.5L V6", for the vehicle's name.
    pub fn name(&self) -> String {
        [
            self.year.clone(),
            title(&self.make),
            self.model.clone(),
            self.engine.clone(),
        ]
        .into_iter()
        .filter(|p| !p.is_empty())
        .collect::<Vec<_>>()
        .join(" ")
    }
}

fn title(s: &str) -> String {
    s.split_whitespace()
        .map(|w| {
            let mut c = w.chars();
            c.next()
                .map(|f| f.to_uppercase().collect::<String>() + &c.as_str().to_lowercase())
                .unwrap_or_default()
        })
        .collect::<Vec<_>>()
        .join(" ")
}

#[derive(Clone, Debug, PartialEq)]
pub struct Recall {
    pub campaign: String,
    pub date: String,
    pub component: String,
    pub summary: String,
    pub consequence: String,
    pub remedy: String,
}

fn fetch(url: &str) -> Result<String, String> {
    if !url.starts_with("https://") {
        return Err("only https".into());
    }
    let out = std::process::Command::new("curl")
        .args([
            "-sS",
            "--fail",
            "--max-time",
            "20",
            "--proto",
            "=https",
            "-A",
            "Icarus vehicle tool",
            url,
        ])
        .output()
        .map_err(|e| format!("curl: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "NHTSA did not answer: {}",
            String::from_utf8_lossy(&out.stderr).trim()
        ));
    }
    String::from_utf8(out.stdout).map_err(|_| "NHTSA's answer is not text".into())
}

pub fn vpic_url(vin: &str) -> String {
    format!(
        "https://vpic.nhtsa.dot.gov/api/vehicles/DecodeVinValues/{}?format=json",
        crate::lookup::encode(vin.trim())
    )
}

pub fn parse_vpic(json: &str) -> Result<VinDecode, String> {
    let v: Value =
        serde_json::from_str(json).map_err(|e| format!("NHTSA's answer could not be read: {e}"))?;
    let r = v
        .get("Results")
        .and_then(|r| r.get(0))
        .ok_or("NHTSA gave no result")?;
    let f = |k: &str| {
        r.get(k)
            .and_then(|x| x.as_str())
            .unwrap_or("")
            .trim()
            .to_string()
    };
    let config = match f("EngineConfiguration").as_str() {
        "V-Shaped" => "V",
        "In-Line" => "I",
        "Horizontally opposed (boxer)" => "H",
        _ => "",
    };
    let engine = match (f("DisplacementL"), f("EngineCylinders")) {
        (d, c) if !d.is_empty() && !c.is_empty() && !config.is_empty() => {
            format!("{}L {config}{c}", trim_number(&d))
        }
        (d, c) if !d.is_empty() && !c.is_empty() => format!("{}L {c}-cylinder", trim_number(&d)),
        (d, _) if !d.is_empty() => format!("{}L", trim_number(&d)),
        _ => String::new(),
    };
    let code = f("ErrorCode");
    Ok(VinDecode {
        make: f("Make"),
        model: f("Model"),
        year: f("ModelYear"),
        trim: f("Trim"),
        engine,
        fuel: f("FuelTypePrimary"),
        drive: f("DriveType"),
        body: f("BodyClass"),
        plant_country: f("PlantCountry"),
        note: (!code.is_empty() && code != "0").then(|| f("ErrorText")),
    })
}

fn trim_number(s: &str) -> String {
    s.parse::<f64>()
        .map(|n| format!("{}", (n * 10.0).round() / 10.0))
        .unwrap_or_else(|_| s.to_string())
}

/// Decodes a VIN at NHTSA (sends the VIN).
pub fn decode_vin(vin: &str) -> Result<VinDecode, String> {
    parse_vpic(&fetch(&vpic_url(vin))?)
}

pub fn recalls_url(make: &str, model: &str, year: u16) -> String {
    use crate::lookup::encode;
    format!(
        "https://api.nhtsa.gov/recalls/recallsByVehicle?make={}&model={}&modelYear={year}",
        encode(make),
        encode(model)
    )
}

pub fn parse_recalls(json: &str) -> Result<Vec<Recall>, String> {
    let v: Value =
        serde_json::from_str(json).map_err(|e| format!("NHTSA's answer could not be read: {e}"))?;
    let list = v
        .get("results")
        .and_then(|r| r.as_array())
        .ok_or("NHTSA gave no recall list")?;
    Ok(list
        .iter()
        .map(|r| {
            let f = |k: &str| {
                r.get(k)
                    .and_then(|x| x.as_str())
                    .unwrap_or("")
                    .trim()
                    .to_string()
            };
            Recall {
                campaign: f("NHTSACampaignNumber"),
                date: f("ReportReceivedDate"),
                component: f("Component"),
                summary: f("Summary"),
                consequence: f("Consequence"),
                remedy: f("Remedy"),
            }
        })
        .collect())
}

/// The recalls on record for a make, model and year (sends only those three).
pub fn recalls(make: &str, model: &str, year: u16) -> Result<Vec<Recall>, String> {
    parse_recalls(&fetch(&recalls_url(make, model, year))?)
}

/// Where to check whether one vehicle (by VIN) is affected and still open.
pub const VIN_RECALL_CHECK: &str = "https://www.nhtsa.gov/recalls";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decoding_and_recalls() {
        // the shape NHTSA answers in (fields as returned on 2026-11-07, values for a made-up VIN)
        let vpic = r#"{"Count":1,"Results":[{"Make":"TOYOTA","Model":"Tacoma","ModelYear":"2017","Trim":"SR5","DisplacementL":"3.500000","EngineCylinders":"6","EngineConfiguration":"V-Shaped","FuelTypePrimary":"Gasoline","DriveType":"4WD/4-Wheel Drive/4x4","BodyClass":"Pickup","PlantCountry":"MEXICO","ErrorCode":"1","ErrorText":"1 - Check Digit (9th position) does not calculate properly"}]}"#;
        let d = parse_vpic(vpic).unwrap();
        assert_eq!(d.name(), "2017 Toyota Tacoma 3.5L V6");
        assert!(d.note.unwrap().contains("Check Digit"));
        let r = parse_recalls(r#"{"Count":1,"Message":"ok","results":[{"NHTSACampaignNumber":"17V831000","ReportReceivedDate":"20/12/2017","Component":"EQUIPMENT:OTHER:LABELS","Summary":"s","Consequence":"c","Remedy":"r"}]}"#).unwrap();
        assert_eq!(
            (r[0].campaign.as_str(), r[0].component.as_str()),
            ("17V831000", "EQUIPMENT:OTHER:LABELS")
        );
        assert_eq!(recalls_url("Toyota", "Tacoma", 2017), "https://api.nhtsa.gov/recalls/recallsByVehicle?make=Toyota&model=Tacoma&modelYear=2017");
        assert!(parse_vpic("not json").is_err() && parse_recalls("{}").is_err());
        assert!(fetch("http://example.com").is_err());
    }
}
