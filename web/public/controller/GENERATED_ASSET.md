# 가상 조종기 프레임

2026-10-11: 배포용 `mango-controller-frame-v3.webp`는 기존 `spider-controller-frame-v2.png`를 `cwebp`으로 압축한 파생 파일이다. 해상도와 투명 배경은 유지했으며 조종기 화면을 열 때만 불러온다. 원본 PNG도 보존했다.

## 대형 디스플레이 프레임 v2

`spider-controller-frame-v2.png`는 v1을 참고 이미지로 넣어 내장 `image_gen` 도구로 편집한 1586×992 투명 PNG다. 좌우 손잡이와 상하 베젤을 줄이고 안테나를 접어 LCD를 확대했다. 이미지의 실제 화면 경계는 x=176…1410, y=139…848이다. CSS에서 y=43…922 범위가 본체 영역에 맞게 표시되도록 투명 여백을 제외한다. 원본 v1과 생성 원본은 보존했다. 기체·지형 화면은 이 이미지 위에 별도로 렌더링하는 WebGL이다.

최종 생성 프롬프트:

```text
Edit this SPIDER drone remote controller product asset into a compact BIG-SCREEN variant for an interactive web app. Preserve its photoreal graphite polymer texture, olive/black defense design, metal knurled thumbsticks, symmetric FRONT view with absolutely no perspective skew, two grips and small orange play button. The user's most important request: radically enlarge the central perfectly rectangular PURE BLACK EMPTY LCD so that it occupies about 74% of the TOTAL IMAGE WIDTH and 76% of the TOTAL IMAGE HEIGHT. Make the left and right handgrips much narrower, keep a small circular thumbstick on each side, drastically shrink the top and bottom bezels. Short folded antennas stay completely inside a slim top strip; NO tall antennas or empty space above the body. Composition: entire device fills 98% of the canvas. Target layout: LCD spans x=13% to 87%, y=12% to 88%; body edges at 1% and 99%. Short antenna tips do not extend beyond the top 10%. Place thumbsticks at x=7% and 93%, y=43%; small three-button stacks on the narrow grips below each stick; power and expand buttons near the bottom corners. The LCD is a single flat EXACT horizontal rectangle with tiny rounded corners, pure black with no reflection, image, glare, text, overlays or interface, because a live WebGL viewport will replace it. Realistic studio highlights on fine textured graphite frame only. Transparent background outside device. Wide 2048x1280 landscape asset. Keep SPIDER as tiny lower-bezel wordmark, no other brand, no extra labels outside the device. The screen must dominate with a very thin realistic bezel. Do not retain the old small screen proportions.
```

## 최초 프레임 v1

`spider-controller-frame-v1.png`는 사용자가 제공한 조종기 사진의 형태를 참고하여 2026-10-10 내장 `image_gen` 도구로 생성했다. 두 안테나, 양쪽 스틱, 중앙 디스플레이와 검은 본체를 가진 가상의 SPIDER 조종기다. DJI 로고와 디스플레이 내용은 사용하지 않는다.

결과는 투명 배경의 1586×992 PNG다. 중앙 화면의 실제 경계는 x=404…1182, y=340…755이며 `remoteController.css`는 이 경계에 WebGL Canvas를 맞춘다. 조종기 본체는 생성한 2D 프레임이고, 그 안의 기체·지형·그림자·프로펠러는 기존 시뮬레이션 상태를 사용하는 실시간 3D다. 사진을 3D 조종기 메시로 복원한 것은 아니다. 스틱과 버튼은 이미지 위의 실제 포인터·키보드 조작 요소다.

최종 생성 프롬프트:

```text
Transform this reference product into a new high-fidelity front-view handheld drone remote controller FRAME asset for a working browser interface. Match the reference's realistic molded black graphite body, two upward folding radio antennas, rubberized grips, two physical metal thumbsticks either side of a large central display, small function buttons, screws, cooling vents, speaker holes and small subtle olive green status LED. Remove all DJI logos and branding and all image content from the display. The entire controller is dead straight frontal, perfectly symmetric horizontal orientation with ZERO perspective skew: the central display opening must be an exact flat horizontal 16:10 rectangle with only tiny corner rounding. This is essential: the interior LCD is a perfectly pure solid black EMPTY rectangle with no graphics, no text, NO reflection and no visible image. We will render a LIVE 3D scene on top of that black LCD. Keep the black screen separate and visually clear, with narrow metal bevels. Frame the whole controller including both antennas and grips entirely in view on a truly TRANSPARENT background, no white or checkerboard background, no environment or cast shadow beyond the hardware. Matte near-black aerospace polymer, very fine realistic microtexture, subtly worn edges, brushed black metal knobs, ultra-real product photography with soft studio highlights and realistic depth. Wide landscape composition 2048x1280. Approximate layout: body spans x=5% to 95%, y=27% to 95%; antennas extend upward into the upper 25%; large rectangular LCD centered at x=25% to 75%, y=35% to 85% (16:10). Put thumbsticks centered at x=15% and 85% flanking the display. Prioritize the rectangular unskewed black LCD and complete visible device. Do not place text, labels or watermarks outside the controller. No branding except a tiny discreet SPIDER wordmark centered on the lower bezel.
```
