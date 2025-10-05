#!/usr/bin/env python3

import json
import re
from typing import Dict, List, Tuple, Optional

class ASLMapper:
    """
    American Sign Language mapping system that translates text to hand gesture configurations.
    Supports finger spelling and common words/phrases.
    """
    
    def __init__(self):
        # Initialize ASL alphabet mappings
        self.asl_alphabet = self._init_asl_alphabet()
        
        # Common word/phrase mappings to avoid finger spelling everything
        self.asl_words = self._init_asl_words()
        
        # Punctuation and special character handling
        self.punctuation_map = self._init_punctuation_map()
        
        # Numbers 0-9 mappings
        self.asl_numbers = self._init_asl_numbers()
    
    def _init_asl_alphabet(self) -> Dict[str, Dict[str, float]]:
        """
        Initialize ASL alphabet hand configurations.
        Each letter maps to joint positions for the robotic hand.
        
        Joint naming convention:
        - thumb: thumb_base_joint, thumb_proximal_joint
        - fingers: {finger}_base_joint, {finger}_proximal_joint, {finger}_middle_joint
        - finger names: index, middle, ring, pinky
        
        Position values: 0.0 = extended, 1.4+ = fully bent
        """
        return {
            'a': {  # Closed fist with thumb on side
                'thumb_base_joint': -0.5, 'thumb_proximal_joint': 0.3,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'b': {  # Four fingers extended, thumb across palm
                'thumb_base_joint': 0.8, 'thumb_proximal_joint': 1.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'c': {  # Curved hand like holding a cup
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.5,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.8, 'index_middle_joint': 0.6,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.8, 'middle_middle_joint': 0.6,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.8, 'ring_middle_joint': 0.6,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.8, 'pinky_middle_joint': 0.6
            },
            'd': {  # Index finger up, others closed, thumb touches middle finger
                'thumb_base_joint': 0.2, 'thumb_proximal_joint': 0.8,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'e': {  # All fingers bent at tips, thumb on side
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.3,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.6, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.6, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.6, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.6, 'pinky_middle_joint': 1.2
            },
            'f': {  # Index and thumb touch, other fingers extended
                'thumb_base_joint': 0.0, 'thumb_proximal_joint': 0.8,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.8, 'index_middle_joint': 1.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'g': {  # Index finger pointing sideways, thumb up
                'thumb_base_joint': -0.8, 'thumb_proximal_joint': 0.0,
                'index_base_joint': -0.5, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'h': {  # Index and middle fingers extended sideways
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': -0.3, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': -0.3, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'i': {  # Pinky finger extended, others closed
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'j': {  # Same as 'i' but with motion (we'll simulate with slight variation)
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.1, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'k': {  # Index up, middle extended sideways, thumb between them
                'thumb_base_joint': -0.2, 'thumb_proximal_joint': 0.6,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': -0.4, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'l': {  # Index finger and thumb extended in L shape
                'thumb_base_joint': -0.8, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'm': {  # Thumb under first three fingers
                'thumb_base_joint': 0.8, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.0, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.0, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.0, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'n': {  # Thumb under first two fingers
                'thumb_base_joint': 0.6, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.0, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.0, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'o': {  # All fingers curved to form a circle
                'thumb_base_joint': 0.0, 'thumb_proximal_joint': 0.8,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.8, 'index_middle_joint': 0.8,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.8, 'middle_middle_joint': 0.8,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.8, 'ring_middle_joint': 0.8,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.8, 'pinky_middle_joint': 0.8
            },
            'p': {  # Same as 'k' but pointing down
                'thumb_base_joint': -0.2, 'thumb_proximal_joint': 0.6,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.3, 'index_middle_joint': 0.0,
                'middle_base_joint': -0.4, 'middle_proximal_joint': 0.3, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'q': {  # Similar to 'g' but pointing down
                'thumb_base_joint': -0.8, 'thumb_proximal_joint': 0.3,
                'index_base_joint': -0.5, 'index_proximal_joint': 0.3, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'r': {  # Index and middle fingers crossed
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.2, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': -0.2, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            's': {  # Closed fist with thumb over fingers
                'thumb_base_joint': 0.0, 'thumb_proximal_joint': 1.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            't': {  # Thumb between index and middle finger
                'thumb_base_joint': 0.2, 'thumb_proximal_joint': 0.8,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.2, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'u': {  # Index and middle fingers extended together
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'v': {  # Index and middle fingers separated in V
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': -0.3, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.3, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'w': {  # Index, middle, and ring fingers extended
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': -0.2, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.2, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'x': {  # Index finger bent at tip
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.2, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            'y': {  # Thumb and pinky extended
                'thumb_base_joint': -0.8, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            'z': {  # Index finger pointing, moving in Z pattern (static representation)
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            }
        }
    
    def _init_asl_numbers(self) -> Dict[str, Dict[str, float]]:
        """Initialize ASL number hand configurations (0-9)"""
        return {
            '0': self.asl_alphabet['o'],  # Same as letter O
            '1': {  # Index finger up
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            },
            '2': self.asl_alphabet['v'],  # Same as letter V
            '3': self.asl_alphabet['w'],  # Same as letter W
            '4': {  # Four fingers extended
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            '5': {  # All fingers extended (open hand)
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            '6': self.asl_alphabet['w'],  # Similar to W but with thumb up
            '7': {  # Ring finger bent down
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            '8': {  # Middle finger bent down
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            },
            '9': {  # Index finger bent down
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            }
        }
    
    def _init_asl_words(self) -> Dict[str, List[Dict[str, float]]]:
        """
        Initialize common ASL word signs.
        Some words have specific signs instead of finger spelling.
        Returns sequences of hand configurations.
        """
        return {
            'hello': [{  # Wave gesture - open hand
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            }],
            'goodbye': [{  # Wave gesture - same as hello but with motion
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            }],
            'please': [{  # Open hand on chest (simulated)
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            }],
            'thank you': [{  # Hand moves from chin down (simulated as open hand)
                'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
                'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
            }],
            'yes': [{  # Fist nodding motion (simulated as fist)
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': 0.0, 'index_proximal_joint': 1.4, 'index_middle_joint': 1.2,
                'middle_base_joint': 0.0, 'middle_proximal_joint': 1.4, 'middle_middle_joint': 1.2,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            }],
            'no': [{  # Index and middle finger closing (like scissors)
                'thumb_base_joint': 0.5, 'thumb_proximal_joint': 1.2,
                'index_base_joint': -0.3, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
                'middle_base_joint': 0.3, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
                'ring_base_joint': 0.0, 'ring_proximal_joint': 1.4, 'ring_middle_joint': 1.2,
                'pinky_base_joint': 0.0, 'pinky_proximal_joint': 1.4, 'pinky_middle_joint': 1.2
            }]
        }
    
    def _init_punctuation_map(self) -> Dict[str, str]:
        """Map punctuation to appropriate pauses or finger spelling"""
        return {
            '.': 'period',
            '!': 'exclamation',
            '?': 'question',
            ',': 'comma',
            ' ': 'space',
            '-': 'dash'
        }
    
    def text_to_gesture_sequence(self, text: str) -> List[Tuple[Dict[str, float], float, str]]:
        """
        Convert text to a sequence of hand gestures.
        
        Args:
            text: Input text to convert
            
        Returns:
            List of tuples: (joint_config, duration, description)
        """
        text = text.lower().strip()
        gesture_sequence = []
        
        # Split into words and handle each
        words = re.findall(r'\b\w+\b|[^\w\s]', text)
        
        for word in words:
            if word in self.asl_words:
                # Use pre-defined word sign
                for config in self.asl_words[word]:
                    gesture_sequence.append((config, 2.0, f"Word: {word}"))
            elif word.isdigit() and len(word) == 1:
                # Single digit number
                config = self.asl_numbers.get(word, self.asl_alphabet.get('a'))
                gesture_sequence.append((config, 1.5, f"Number: {word}"))
            else:
                # Finger spell the word
                for char in word:
                    if char in self.asl_alphabet:
                        config = self.asl_alphabet[char]
                        gesture_sequence.append((config, 1.0, f"Letter: {char}"))
                    elif char.isdigit() and char in self.asl_numbers:
                        config = self.asl_numbers[char]
                        gesture_sequence.append((config, 1.0, f"Number: {char}"))
            
            # Add pause between words
            if word != words[-1]:  # Not the last word
                pause_config = self._get_neutral_position()
                gesture_sequence.append((pause_config, 0.5, "Word pause"))
        
        return gesture_sequence
    
    def _get_neutral_position(self) -> Dict[str, float]:
        """Return neutral hand position"""
        return {
            'thumb_base_joint': -0.3, 'thumb_proximal_joint': 0.0,
            'index_base_joint': 0.0, 'index_proximal_joint': 0.0, 'index_middle_joint': 0.0,
            'middle_base_joint': 0.0, 'middle_proximal_joint': 0.0, 'middle_middle_joint': 0.0,
            'ring_base_joint': 0.0, 'ring_proximal_joint': 0.0, 'ring_middle_joint': 0.0,
            'pinky_base_joint': 0.0, 'pinky_proximal_joint': 0.0, 'pinky_middle_joint': 0.0
        }
    
    def get_gesture_for_text(self, text: str) -> Optional[Dict[str, float]]:
        """
        Get a single gesture configuration for simple text.
        Useful for single letters, numbers, or simple words.
        """
        text = text.lower().strip()
        
        if text in self.asl_words:
            return self.asl_words[text][0]  # Return first configuration
        elif text in self.asl_alphabet:
            return self.asl_alphabet[text]
        elif text in self.asl_numbers:
            return self.asl_numbers[text]
        else:
            return None
    
    def get_available_gestures(self) -> Dict[str, List[str]]:
        """Return available gestures organized by category"""
        return {
            'alphabet': list(self.asl_alphabet.keys()),
            'numbers': list(self.asl_numbers.keys()),
            'words': list(self.asl_words.keys())
        }
    
    def save_custom_gesture(self, name: str, config: Dict[str, float], category: str = 'custom'):
        """Save a custom gesture configuration"""
        if category == 'words':
            self.asl_words[name] = [config]
        elif category == 'alphabet':
            self.asl_alphabet[name] = config
        elif category == 'numbers':
            self.asl_numbers[name] = config
        # Could extend to save to file for persistence