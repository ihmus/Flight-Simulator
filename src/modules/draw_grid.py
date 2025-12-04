def draw_grid(self):
        glDisable(GL_LIGHTING)
        glColor3f(0.2,0.2,0.2)
        glBegin(GL_LINES)
        for i in range(-10,11):
            glVertex3f(i,-1.0,-10); glVertex3f(i,-1.0,10)
            glVertex3f(-10,-1.0,i); glVertex3f(10,-1.0,i)
        glEnd()
