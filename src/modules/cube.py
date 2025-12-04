    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Kamerayı ayarla
        eye, target, up = self._compute_camera_view()
        gluLookAt(eye[0],eye[1],eye[2], target[0],target[1],target[2], up[0],up[1],up[2])


        # Küpü çiz
        self.draw_cube()

    def draw_cube(self):
        # Küpün köşeleri
        vertices = [
            (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1), (-1, -1, 1), (-1, 1, 1)
        ]
        
        # Küpün yüzleri (her bir yüzün köşeleri)
        faces = [
            (0, 1, 2, 3),  # Arka yüz
            (4, 5, 6, 7),  # Ön yüz
            (0, 1, 5, 4),  # Sağ yüz
            (2, 3, 7, 6),  # Sol yüz
            (1, 2, 6, 5),  # Üst yüz
            (0, 3, 7, 4)   # Alt yüz
        ]
        
        # Her yüz için renkler
        colors = [
            (1, 0, 0),  # Kırmızı
            (0, 1, 0),  # Yeşil
            (0, 0, 1),  # Mavi
            (1, 1, 0),  # Sarı
            (0, 1, 1),  # Cyan
            (1, 0, 1)   # Magenta
        ]

        # Yüzleri çiz
        for i, face in enumerate(faces):
            glBegin(GL_QUADS)  # Her yüz için dörtgen çiz
            glColor3fv(colors[i])  # Yüz için rengi ayarla
            for vertex in face:
                glVertex3fv(vertices[vertex])
            glEnd()

        # Kenarları çiz
        glBegin(GL_LINES)  # Kenarları çiz
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()
