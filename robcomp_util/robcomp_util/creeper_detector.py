from robcomp_util.module_aruco import Aruco3d
import cv2
import numpy as np

class CreeperDetector(Aruco3d):
    def __init__(self):
        super().__init__()
        self.kernel = np.ones((5,5), np.uint8)

        # filtros HSV para cada cor
        # vermelho precisa de duas faixas (wrap-around do hue)
        self.filters = {
            'red': {
                'lower1': np.array([  0, 100, 100]),
                'upper1': np.array([ 10, 255, 255]),
                'lower2': np.array([165, 100, 90]),
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([100,  50,  0]),
                'upper': np.array([140, 255, 255])
            },
            'green': {
                'lower': np.array([ 40,  0,  0]),
                'upper': np.array([ 80, 255, 60])
            }
        }

    def find_creeper(self, bgr: np.ndarray, color: str) -> list:
        """
        Encontra creepers de uma cor específica usando segmentação HSV.
        Retorna lista de [ (cx, cy), color ].
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        if color == 'red':
            # mascara para vermelho em duas faixas de Hue
            m1 = cv2.inRange(hsv,
                             self.filters['red']['lower1'],
                             self.filters['red']['upper1'])
            m2 = cv2.inRange(hsv,
                             self.filters['red']['lower2'],
                             self.filters['red']['upper2'])
            mask = cv2.bitwise_or(m1, m2)
        else:
            lo = self.filters[color]['lower']
            hi = self.filters[color]['upper']
            mask = cv2.inRange(hsv, lo, hi)

        # limpeza morfológica
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # encontra contornos e filtra por área
        contours, _ = cv2.findContours(mask,
                                       cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        creepers = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 1000:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w//2, y + h//2
            creepers.append([(cx, cy), color])

        return creepers

    def distance(self, x1, x2):
        """Distância horizontal absoluta."""
        return abs(x1 - x2)

    def match_aruco(self, bgr, creepers, results):
        """
        Combina marcadores ArUco com centros de creepers.
        Adiciona em cada `cabeca` as chaves 'body_center' e 'color'.
        """
        matched_pairs = []
        for cabeca in results:
            # escolhe o creeper mais próximo em x
            closest = min(creepers,
                          key=lambda cr: self.distance(cabeca['centro'][0], cr[0][0]))
            # remove para não reusar
            creepers = [cr for cr in creepers if cr[0] != closest[0]]
            cabeca['body_center'] = closest[0]
            cabeca['color']       = closest[1]
            cv2.line(bgr,
                     tuple(cabeca['centro']),
                     cabeca['body_center'],
                     (0,0,255), 2)
            matched_pairs.append(cabeca)
            if not creepers:
                break
        return bgr, matched_pairs

    def run(self, bgr):
        """
        1) detecta ArUcos
        2) encontra creepers em vermelho, azul e verde
        3) faz o match e desenha
        4) retorna lista de dicts com keys:
           ['id','rvec','tvec','distancia','corners','centro',
            'body_center','color']
        """
        # 1) detecta ArUcos na imagem
        _, results = self.detectaAruco(bgr)

        # 2) detecta creepers de cada cor
        creepers = []
        for color in ('red', 'blue', 'green'):
            creepers += self.find_creeper(bgr, color)

        if not creepers or not results:
            return bgr, []

        # 3) emparelha
        bgr, matched = self.match_aruco(bgr, creepers, results)

        # 4) desenha cada ArUco encontrado
        for r in matched:
            bgr = self.drawAruco(bgr, r)

        # 5) adiciona creepers sem ArUco
        for cx, color in creepers:
            matched.append({
                'body_center': cx,
                'color': color,
                'id': 0
            })

        return bgr, matched


def main():
    Arucos = CreeperDetector()
    img = cv2.imread("robcomp_util/robcomp_util/aruco.jpeg")
    bgr, ranked = Arucos.run(img)
    print(ranked)
    cv2.imshow("Detecção de Creepers", bgr)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
