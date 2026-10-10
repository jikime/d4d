import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./styles.css";
import App from "./App";
import "./hardwareUX.css";
import "./defense.css";
import "./formControls.css";
import AppErrorBoundary from "./AppErrorBoundary";
import {LanguageMetadata} from "./i18n/LanguageControls";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <LanguageMetadata />
    <AppErrorBoundary><App /></AppErrorBoundary>
  </StrictMode>,
);
