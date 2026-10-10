# 합성 시험장 지형 이미지

2026-10-11: 배포용 `field-ground-albedo-v2.webp`는 기존 `field-ground-albedo-v1.png`를 `cwebp`으로 압축한 파생 파일이다. 지형 내용과 해상도는 유지했으며 원본 PNG도 보존했다.

## 야외 훈련장 표면 재질

`field-ground-albedo-v1.png`는 2026-10-10 내장 `image_gen` 도구로 생성한 흙·자갈의 베이스 컬러 텍스처다. `web/src/simulation/sceneTerrain.ts`의 높낮이가 있는 3D 메시 표면에 반복 적용한다. 이미지를 3D로 복원한 모델은 아니며, 지형 형상·풀·수목·시설·먼지는 독립적인 실시간 3D 객체다. 실제 훈련장이나 측량 자료를 나타내지 않는다. 별도의 높이·법선 자료가 없는 단일 컬러 재질이며, 미세 요철은 코드에서 생성한 범프 맵으로 보완했다.

최종 생성 프롬프트:

```text
Use case: photorealistic-natural. Asset type: seamless tileable PBR base-color ground texture for a real-time Three.js outdoor drone training range. Create a square 1536x1536 photorealistic perfectly orthographic top-down material scan of compacted gray-brown earth, fine worn gravel, small natural stone chips, irregular dry soil grains and a very sparse amount of tiny dry straw. Neutral desaturated earth gray and brown palette, subtle broad mottling and high frequency natural microdetail. Flat diffuse shadowless overcast lighting; this is an ALBEDO map, no baked directional shadows or highlights. Edge-to-edge seamless repeating material, equal density across all quadrants, no obvious repeating features, no horizon, no perspective, no buildings, no people, no drones, no plants growing upright, no writing, no logo, no border, no large rocks, no green tint, no artistic illustration.
```

## 메인 화면 항공사진

`spider-ortho-v2.png`는 2026-10-10 기본 내장 이미지 생성 도구로 만든 가상 시험장 항공사진이다. 실제 위치·장애물·좌표를 나타내지 않는다. 화면에서 시나리오의 로컬 좌표를 덧그리는 배경으로만 사용한다. 원본 자산은 유지했다.

최종 생성 프롬프트:

```text
Use case: photorealistic-natural
Asset type: high-resolution orthographic aerial terrain background for a fictional drone simulation command interface.
Primary request: a completely photoreal drone orthomosaic of a quiet small industrial field research campus, rectangular 40 by 24 meter scene, in a wide 16:10 composition. Strictly straight down orthographic view, no horizon, no perspective tilt. Weathered light gray concrete test yard and gravel access road occupy most of the central area. Several small low buildings with gray metal roofs near the edges, two orderly equipment containers, sparse trees and muted green vegetation around the perimeter. A few realistic expansion joints and worn tire traces, realistic sharp material detail. No people, no aircraft, no weapons.
Style: professionally captured high resolution aerial photography, subtle filmic grading, detailed concrete and vegetation, natural late-morning light with soft directional shadows, blue-gray/olive/earth tones, well exposed and clear rather than dark.
Composition: useful large unobstructed center for separately rendered mission markers and routes. Buildings confined to upper left, upper right and lower edge. No baked graphical overlays.
Constraints: no text, no labels, no icons, no numbers, no grid, no UI, no branding, no watermark. This is a fictional simulation terrain asset, not a real location. Produce landscape image about 2048 by 1280.
```
