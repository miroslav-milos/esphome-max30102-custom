# ESPHome MAX30102 Custom Component (Advanced HR + SpO₂ + Finger Detect)

Ovo je napredni MAX30102 driver za ESPHome koji uključuje:

- Napredni algoritam (Model B)
- HR detekcija s median filtrom (5 beat intervala)
- SpO₂ s median 7‑sample windowom
- Perfusion Index (PI) gating
- Motion artefact rejection
- Hybrid finger detect (IR threshold + PI)
- Direktna podrška za LED kontrolu (idle/active)
- Runtime podešavanje sample rate / averaging / ADC range

## 📦 Instalacija

```yaml
external_components:
  - source: github://miroslav-milos/esphome-max30102-custom@main
    refresh: 0s
    components: [max30102_custom]