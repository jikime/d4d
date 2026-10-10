import {t} from "./i18n";
import { Component, type ErrorInfo, type ReactNode } from "react";
import { RefreshCw, ShieldAlert } from "lucide-react";

export default class AppErrorBoundary extends Component<{ children: ReactNode }, { failed: boolean }> {
  state = { failed: false };
  static getDerivedStateFromError() { return { failed: true }; }
  componentDidCatch(error: Error, info: ErrorInfo) { console.error("MANGO 화면 오류", error, info.componentStack); }
  render() {
    if (!this.state.failed) return this.props.children;
    return <main className="app-recovery" role="alert">
      <ShieldAlert size={36} aria-hidden="true" />
      <h1>{t("화면을 다시 연결해야 합니다")}</h1>
      <p>{t("예기치 않은 화면 오류가 발생했습니다. 저장한 임무는 라이브러리에 유지됩니다. 저장하지 않은 수정은 새로고침하면 사라질 수 있습니다.")}</p>
      <button className="button primary" onClick={() => window.location.reload()}><RefreshCw size={16} />{t(" 화면 새로고침")}</button>
      <small>{t("실기기 상태는 화면 복구 뒤 새 보고로 확인하세요.")}</small>
    </main>;
  }
}
