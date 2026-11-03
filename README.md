# DD-NMPC Browser Playground — GitHub Pages

Static browser-only demo of the Panamax distance-domain NMPC prototype. CasADi 3.8.1 WebAssembly and FATROP execute inside the visitor's browser; no Python service is deployed.

## Publish

1. Create a **public** GitHub repository (for example, `dd-nmpc-playground`) in your account.
2. Upload **the contents of this directory** into the repository root, including `index.html`, `app.js`, `styles.css`, `wasm/`, and `.nojekyll`.
3. In **Settings → Pages**, choose **Deploy from a branch**, branch **main**, folder **/(root)**, and save.
4. Open `https://YOUR-USERNAME.github.io/YOUR-REPOSITORY/` after the Pages deployment completes.

For local testing, execute in this directory:

```sh
python3 -m http.server 8765 --bind 127.0.0.1
```

Then open `http://127.0.0.1:8765/`. Do not use `--directory playground` from within this directory.

## Functionality and limitations

- Three route presets; compiled shooting/control-block pairs N/M = 8/2, 8/4, 12/4.
- Editable horizon, RPM, initial errors, environmental current, number of cycles.
- Reference track, nominal WPs, applied trajectory, predicted MPC track, input histories and predicted future inputs.
- First run downloads the CasADi WebAssembly runtime from unpkg.com; internet access and browser WebAssembly support are required.
- The nine compiled `wasm/*.json` graphs contain publicly downloadable research-model data. Do not publish them if the model needs to remain confidential.
- Runs are numerical model-in-the-loop demonstrations; predictions are not measurements, and the system is not validated/certified for operational use.
