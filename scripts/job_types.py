#!/usr/bin/env python3
"""
Job Types - 태스크 시퀀스에서 사용하는 작업 타입 정의

각 작업 타입은 다음 구조를 따릅니다:
- name: 표시 이름
- category: 카테고리 (트리 뷰 그룹핑용)
- params: 파라미터 정의 (타입, 기본값, 단위 등)
- has_read_position: "현재 위치 읽기" 버튼 표시 여부
"""

JOB_TYPES = {
    # Motion
    'go_home': {
        'name': 'Go Home',
        'category': 'Motion',
        'params': {}
    },
    'move_to_pose': {
        'name': '위치 이동',
        'category': 'Motion',
        'params': {
            'x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 위치'},
            'y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 위치'},
            'z': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Z 위치'},
            'rx': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rx 회전'},
            'ry': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Ry 회전'},
            'rz': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rz 회전'},
        },
        'has_read_position': True
    },
    'tcp_linear_x': {
        'name': 'TCP Linear X',
        'category': 'Motion',
        'params': {
            'coordinate': {'type': 'str', 'default': 'TF1', 'options': ['Base', 'TF1', 'TF2', 'TF3', 'TF4', 'TF5'], 'description': '좌표계'},
            'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '이동 모드'},
            'distance': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 이동 거리', 'description_absolute': 'X 목표 위치'},
        }
    },
    'tcp_linear_y': {
        'name': 'TCP Linear Y',
        'category': 'Motion',
        'params': {
            'coordinate': {'type': 'str', 'default': 'TF1', 'options': ['Base', 'TF1', 'TF2', 'TF3', 'TF4', 'TF5'], 'description': '좌표계'},
            'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '이동 모드'},
            'distance': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 이동 거리', 'description_absolute': 'Y 목표 위치'},
        }
    },
    'tcp_linear_z': {
        'name': 'TCP Linear Z',
        'category': 'Motion',
        'params': {
            'coordinate': {'type': 'str', 'default': 'TF1', 'options': ['Base', 'TF1', 'TF2', 'TF3', 'TF4', 'TF5'], 'description': '좌표계'},
            'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '이동 모드'},
            'distance': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Z 이동 거리', 'description_absolute': 'Z 목표 위치'},
        }
    },
    'tcp_linear_xyz': {
        'name': 'TCP Linear XYZ',
        'category': 'Motion',
        'params': {
            'coordinate': {'type': 'str', 'default': 'TF1', 'options': ['Base', 'TF1', 'TF2', 'TF3', 'TF4', 'TF5'], 'description': '좌표계'},
            'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '이동 모드'},
            'x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 이동 거리', 'description_absolute': 'X 목표 위치'},
            'y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 이동 거리', 'description_absolute': 'Y 목표 위치'},
            'z': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Z 이동 거리', 'description_absolute': 'Z 목표 위치'},
        }
    },
    'tcp_rotate_rx': {
        'name': 'TCP Rotate Rx',
        'category': 'Motion',
        'params': {
            'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '회전 모드'},
            'angle': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rx 회전 각도', 'description_absolute': 'Rx 목표 각도'},
        }
    },
    'tcp_rotate_ry': {
        'name': 'TCP Rotate Ry',
        'category': 'Motion',
        'params': {
            'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '회전 모드'},
            'angle': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Ry 회전 각도', 'description_absolute': 'Ry 목표 각도'},
        }
    },
    'tcp_rotate_rz': {
        'name': 'TCP Rotate Rz',
        'category': 'Motion',
        'params': {
            'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '회전 모드'},
            'angle': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rz 회전 각도', 'description_absolute': 'Rz 목표 각도'},
        }
    },
    'tcp_rotate_rxryrz': {
        'name': 'TCP Rotate RxRyRz',
        'category': 'Motion',
        'params': {
            'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '회전 모드'},
            'rx': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rx 회전 각도', 'description_absolute': 'Rx 목표 각도'},
            'ry': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Ry 회전 각도', 'description_absolute': 'Ry 목표 각도'},
            'rz': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rz 회전 각도', 'description_absolute': 'Rz 목표 각도'},
        }
    },

    # Vision
    'detect_aruco': {
        'name': 'Aruco Tag 인식',
        'category': 'Vision',
        'params': {
            'tag_id': {'type': 'int', 'default': 0, 'description': 'Tag ID'},
            'timeout': {'type': 'float', 'default': 10.0, 'unit': 'sec'},
        }
    },
    'detect_dual_aruco_plane': {
        'name': '듀얼 ArUco 평면 추출',
        'category': 'Vision',
        'params': {
            'tag_id1': {'type': 'int', 'default': 0, 'min': 0, 'max': 255, 'description': 'Tag ID 1'},
            'tag_id2': {'type': 'int', 'default': 1, 'min': 0, 'max': 255, 'description': 'Tag ID 2'},
            'num_samples': {'type': 'int', 'default': 20, 'min': 5, 'max': 100, 'description': '인식 횟수'},
            'delay_ms': {'type': 'int', 'default': 100, 'min': 0, 'max': 1000, 'unit': 'ms', 'description': '샘플 간 지연'},
            'timeout': {'type': 'float', 'default': 30.0, 'unit': 'sec', 'description': '타임아웃'},
        },
        'has_result_display': True,
    },

    # 정렬 (Alignment)
    'align_aruco_center': {
        'name': 'Aruco Tag 중심 정렬',
        'category': '정렬',
        'params': {
            'tag_id': {'type': 'int', 'default': 0, 'description': 'Tag ID'},
            'offset_x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 오프셋'},
            'offset_y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 오프셋'},
        }
    },
    'align_aruco_pose': {
        'name': 'Aruco Tag 자세 정렬',
        'category': '정렬',
        'params': {
            'tag_id1': {'type': 'int', 'default': 0, 'min': 0, 'max': 255, 'description': 'Tag ID 1'},
            'tag_id2': {'type': 'int', 'default': 1, 'min': 0, 'max': 255, 'description': 'Tag ID 2'},
            'num_samples': {'type': 'int', 'default': 20, 'min': 5, 'max': 100, 'description': '인식 횟수'},
            'target_rx': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': '목표 Rx'},
            'target_ry': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': '목표 Ry'},
            'target_rz': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': '목표 Rz'},
            'timeout': {'type': 'float', 'default': 30.0, 'unit': 'sec', 'description': '타임아웃'},
        },
        'has_result_display': True,
    },
    'align_aruco_full': {
        'name': 'Aruco Tag 전체 정렬',
        'category': '정렬',
        'params': {
            'tag_id': {'type': 'int', 'default': 0, 'description': 'Tag ID'},
            'offset_x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 오프셋'},
            'offset_y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 오프셋'},
        }
    },
    'tcp_offset_move': {
        'name': 'TCP 오프셋 이동',
        'category': '정렬',
        'params': {
            'offset_x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 오프셋'},
            'offset_y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 오프셋'},
            'offset_z': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Z 오프셋'},
        }
    },

    # Gripper
    'gripper': {
        'name': '그리퍼',
        'category': 'Gripper',
        'params': {
            'action': {'type': 'str', 'default': 'close', 'options': ['open', 'close', 'home']},
        }
    },

    # Modbus
    'write_modbus': {
        'name': 'Modbus 쓰기',
        'category': 'Modbus',
        'params': {
            'register': {'type': 'int', 'default': 351},
            'value': {'type': 'int', 'default': 0},
        }
    },
    'send_response': {
        'name': '응답 전송',
        'category': 'Modbus',
        'params': {
            'response': {'type': 'str', 'default': 'pose_back', 'options': ['pose_back', 'pose_main']},
        }
    },

    # Control
    'wait': {
        'name': '대기',
        'category': 'Control',
        'params': {
            'duration': {'type': 'int', 'default': 500, 'unit': 'msec', 'min': 10, 'max': 10000, 'step': 10},
        }
    },

    # Frame
    'toolframe': {
        'name': '툴프레임 변경',
        'category': 'Frame',
        'params': {
            'frame': {'type': 'int', 'default': 0, 'min': 0, 'max': 3, 'description': '툴프레임 번호 (0-3)'},
        }
    },
    'base': {
        'name': '베이스프레임 초기화',
        'category': 'Frame',
        'params': {}
    },
}
