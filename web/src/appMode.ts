// 공개 정적 배포에서만 활성화한다. 로컬 개발/일반 빌드는 기존 API와 기체 연결을 사용한다.
export const DEMO_MODE = import.meta.env.VITE_DEMO_MODE === 'true';
