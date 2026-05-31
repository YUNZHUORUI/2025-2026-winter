# Solift H-VTOL Final Report — Overleaf LaTeX Project
## TECHPLANE001 | Technion DFM Course

---

## How to Upload and Compile on Overleaf

1. Go to **overleaf.com** → New Project → Upload Project
2. Upload the **entire ZIP file** (all files including the `sections/` and `figures/` folders)
3. Set the **main document** to `main.tex`
4. Set the **compiler** to **pdfLaTeX** (Project Settings → Compiler)
5. Click **Recompile**

---

## Project Structure

```
solift_report/
├── main.tex                    ← Master document (compile this)
├── references.bib              ← BibTeX references
├── README.md                   ← This file
├── sections/
│   ├── 01_object_selection.tex ← Chapter 1: Object Selection
│   ├── 02_comparative_matrix.tex ← Chapter 2: Weighted Matrix
│   ├── 03_srr.tex              ← Chapter 3: SRR Document
│   ├── 04_concept_design.tex   ← Chapter 4: 5-Phase Design
│   ├── 05_tech_specs.tex       ← Chapter 5: Technical Specs
│   ├── 06_pdr.tex              ← Chapter 6: PDR Document
│   └── 07_appendix.tex         ← Chapter 7: Appendix
└── figures/
    ├── technion_logo.png       ← ADD: Technion logo
    ├── solift_render.png       ← ADD: CAD isometric render (Image 1)
    ├── solift_ground.png       ← ADD: Ground render with payload (Image 2)
    ├── solift_cad_iso.png      ← ADD: SolidWorks isometric screenshot
    ├── solift_cad_explode.png  ← ADD: SolidWorks exploded view
    └── tradeoff_range.png      ← ADD: Range trade-off chart (Excel/MATLAB)
```

---

## Figures You Need to Add

Upload these images to the `figures/` folder in Overleaf:

| Filename | Content | Source |
|---|---|---|
| `technion_logo.png` | Technion official logo | Technion website |
| `solift_render.png` | CAD isometric render (Image 1 from your project) | Your SolidWorks / render |
| `solift_ground.png` | Ground render with payload bay open (Image 2) | Your render |
| `solift_cad_iso.png` | SolidWorks isometric screenshot | SolidWorks 2024 |
| `solift_cad_explode.png` | SolidWorks exploded view of tilt sub-assy | SolidWorks 2024 |
| `tradeoff_range.png` | Range vs. battery/solar trade-off chart | Excel / MATLAB / Python |

> **Tip:** If a figure is not yet ready, comment out the `\includegraphics` line and replace with `\fbox{\parbox{0.8\textwidth}{\centering [Figure placeholder]}}` temporarily.

---

## Fill-In Checklist

Search for `[...]` placeholders throughout the `.tex` files and replace:

- `[Course Name]` → Your actual course name
- `[Name 1]`, `[Name 2]`, etc. → Group member names
- `[Student ID]` → Student IDs
- `[Date]` → Submission date
- `[Instructor Name]` → Your professor's name
- `[Team Name]` → Your team name

---

## Packages Required

All packages used are available in Overleaf's default TeX Live installation. No additional installations needed.

Key packages: `geometry`, `booktabs`, `longtable`, `tabularx`, `amsmath`, `siunitx`, `hyperref`, `graphicx`, `pgfplots`, `natbib`

---

## Estimated Page Count

| Section | Estimated Pages |
|---|---|
| Title page, abstract, TOC | 5–6 |
| Chapter 1: Object Selection | 6–8 |
| Chapter 2: Comparative Matrix | 4–5 |
| Chapter 3: SRR | 6–8 |
| Chapter 4: Detailed Design (5 phases) | 20–25 |
| Chapter 5: Technical Specifications | 5–6 |
| Chapter 6: PDR | 6–8 |
| Chapter 7: Appendix | 10–15 |
| **Total** | **~62–75 pages** |

This meets the 60–70 page requirement. Add more figures and expand narrative sections as needed.
