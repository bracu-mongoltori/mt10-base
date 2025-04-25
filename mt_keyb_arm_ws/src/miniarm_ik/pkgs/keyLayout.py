import cv2, numpy as np

def bindLayout(frame, target_key, head_pos, tracking_points, tracking, wPoints, kP, params):
    cv2.putText(frame, f"Target Key: {target_key}", (10, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "Keybinds:", (10, 50), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "any key: move to key and stop tracking", (10, 70), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "q: quit", (10, 90), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "s: save screen points", (10, 110), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "r: restart adding points", (10, 130), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "l: load saved points", (10, 150), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "c: resume tracking", (10, 170), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "i: initiate typing sequence", (10, 190), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    if len(wPoints) == 4:
        cv2.drawContours(frame, [np.array(wPoints)], -1, (0, 255, 255), 1, cv2.LINE_AA)
    for p in wPoints:
        cv2.circle(frame, (int(p[0]), int(p[1])), 8, (0, 0, 255), 2, cv2.LINE_AA)

    if(len(tracking_points) == 4):
        for key in params.key_pos:
            sPos = kP.getScrPos(key)
            if sPos is None:
                continue
            font_scale = 0.75
            text_size = cv2.getTextSize(key, cv2.FONT_HERSHEY_COMPLEX_SMALL, font_scale, 1)[0]
            _x = int(sPos[0] - text_size[0] / 2)
            _y = int(sPos[1] + text_size[1] / 2)
            cv2.putText(
                frame, 
                key, 
                (_x, _y), 
                cv2.FONT_HERSHEY_COMPLEX_SMALL, 
                font_scale, 
                (255, 0, 255) if key == target_key else (255, 255, 0),
                1, 
                cv2.LINE_AA
            )

        cv2.drawContours(frame, [np.array(tracking_points)], -1, (255, 255, 0), 2, cv2.LINE_AA)
    
    if tracking:
        for p in tracking_points:
            cv2.circle(frame, (p[0], p[1]), 5, (0, 255, 0), -1, cv2.LINE_AA)
            
    if(head_pos is not None and len(tracking_points) == 4 and len(wPoints) == 4):
        sPos = kP.getPfrW(head_pos)
        cv2.circle(frame, (int(sPos[0]), int(sPos[1])), 10, (255, 0, 255), 2, cv2.LINE_AA)
