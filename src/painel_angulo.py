"""Painel em tela cheia com o angulo atual, para acompanhar o ensaio pela
tela do PC em vez do OLED. Le a mesma telemetria do coleta_degrau.py.

    python3 painel_angulo.py [porta]     ESC sai | F tela cheia | D tema
"""

import sys
import threading
import tkinter as tk

import serial
from serial.tools import list_ports

PORTA_PADRAO = '/dev/ttyUSB0'
BAUD_RATE = 115200
INTERVALO_TELA_MS = 50                                                          # 20 quadros por segundo na tela

TEMAS = {
    'claro': {'fundo': 'white', 'angulo': 'black', 'apoio': '#555555',
              'alvo': '#c00000', 'erro': '#0050c0', 'off': '#999999'},
    'escuro': {'fundo': 'black', 'angulo': 'white', 'apoio': '#aaaaaa',
               'alvo': '#ff6666', 'erro': '#66aaff', 'off': '#666666'},
}


def descobrir_porta():
    """Usa a porta passada na linha de comando ou procura um conversor USB."""
    if len(sys.argv) > 1:
        return sys.argv[1]
    for p in list_ports.comports():
        if 'USB' in p.device or 'ACM' in p.device:
            return p.device
    return PORTA_PADRAO


class LeitorSerial(threading.Thread):
    """Le a serial em segundo plano e guarda sempre a ultima amostra."""

    def __init__(self, porta):
        super().__init__(daemon=True)
        self.porta = porta
        self.amostra = None                                                     # (tempo_ms, angulo, alvo)
        self.conectado = False
        self.erro = ''
        self._parar = threading.Event()

    def parar(self):
        self._parar.set()

    def run(self):
        while not self._parar.is_set():
            try:
                with serial.Serial(self.porta, BAUD_RATE, timeout=1) as ser:
                    ser.reset_input_buffer()
                    self.conectado = True
                    self.erro = ''
                    while not self._parar.is_set():
                        linha = ser.readline().decode('utf-8', errors='ignore').strip()
                        partes = linha.split(',')
                        if len(partes) != 3:
                            continue
                        try:
                            self.amostra = (int(partes[0]), float(partes[1]), float(partes[2]))
                        except ValueError:
                            pass                                                # linha truncada: ignora
            except serial.SerialException as e:
                self.conectado = False
                self.erro = str(e)
                self._parar.wait(1.0)                                           # tenta reconectar em 1 s


class Painel:
    def __init__(self, leitor):
        self.leitor = leitor
        self.tema = 'claro'
        self.cheia = True

        self.raiz = tk.Tk()
        self.raiz.title(f'Aeropendulo — {leitor.porta}')
        self.raiz.attributes('-fullscreen', True)
        self.raiz.configure(bg=TEMAS[self.tema]['fundo'])

        self.tela = tk.Canvas(self.raiz, highlightthickness=0, bd=0)
        self.tela.pack(fill='both', expand=True)

        # Os itens sao criados uma vez e so tem texto/posicao atualizados.
        self.id_angulo = self.tela.create_text(0, 0, text='--', anchor='c')
        self.id_rotulo = self.tela.create_text(0, 0, text='ANGULO', anchor='c')
        self.id_alvo = self.tela.create_text(0, 0, text='', anchor='c')
        self.id_status = self.tela.create_text(0, 0, text='', anchor='c')

        self.tela.bind('<Configure>', lambda e: self.posicionar(e.width, e.height))
        for tecla in ('<Escape>', 'q', 'Q'):
            self.raiz.bind(tecla, lambda e: self.sair())
        self.raiz.bind('<f>', lambda e: self.alternar_cheia())
        self.raiz.bind('<F>', lambda e: self.alternar_cheia())
        self.raiz.bind('<d>', lambda e: self.alternar_tema())
        self.raiz.bind('<D>', lambda e: self.alternar_tema())
        self.raiz.protocol('WM_DELETE_WINDOW', self.sair)

        self.aplicar_tema()
        self.atualizar()

    def posicionar(self, largura, altura):
        """Reescala fontes e posicoes conforme o tamanho da janela."""
        base = min(largura / 6.2, altura / 2.8)                                 # "-100.0" ocupa ~80% da largura
        grande = max(int(base), 12)
        medio = max(int(base * 0.16), 10)

        self.tela.coords(self.id_rotulo, largura / 2, altura * 0.20)
        self.tela.itemconfig(self.id_rotulo, font=('DejaVu Sans', medio))

        self.tela.coords(self.id_angulo, largura / 2, altura * 0.48)
        self.tela.itemconfig(self.id_angulo, font=('DejaVu Sans', grande, 'bold'))

        self.tela.coords(self.id_alvo, largura / 2, altura * 0.78)
        self.tela.itemconfig(self.id_alvo, font=('DejaVu Sans', int(medio * 1.3)))

        self.tela.coords(self.id_status, largura / 2, altura * 0.94)
        self.tela.itemconfig(self.id_status, font=('DejaVu Sans', max(int(medio * 0.6), 9)))

    def aplicar_tema(self):
        cores = TEMAS[self.tema]
        self.raiz.configure(bg=cores['fundo'])
        self.tela.configure(bg=cores['fundo'])
        self.tela.itemconfig(self.id_angulo, fill=cores['angulo'])
        self.tela.itemconfig(self.id_rotulo, fill=cores['apoio'])
        self.tela.itemconfig(self.id_status, fill=cores['apoio'])

    def alternar_tema(self):
        self.tema = 'escuro' if self.tema == 'claro' else 'claro'
        self.aplicar_tema()

    def alternar_cheia(self):
        self.cheia = not self.cheia
        self.raiz.attributes('-fullscreen', self.cheia)

    def atualizar(self):
        cores = TEMAS[self.tema]
        amostra = self.leitor.amostra

        if amostra is None:
            self.tela.itemconfig(self.id_angulo, text='--', fill=cores['off'])
            self.tela.itemconfig(self.id_alvo, text='')
        else:
            tempo_ms, angulo, alvo = amostra
            self.tela.itemconfig(self.id_angulo, text=f'{angulo:.1f}', fill=cores['angulo'])
            if alvo != 0.0:                                                     # malha fechada: alvo e erro
                self.tela.itemconfig(
                    self.id_alvo,
                    text=f'alvo {alvo:.1f}°     erro {alvo - angulo:+.1f}°',
                    fill=cores['erro'])
            else:                                                               # malha aberta: so o relogio da placa
                self.tela.itemconfig(self.id_alvo,
                                     text=f'{tempo_ms / 1000.0:.1f} s',
                                     fill=cores['apoio'])

        if self.leitor.conectado:
            status = f'{self.leitor.porta} @ {BAUD_RATE}   —   ESC sai · F tela cheia · D tema'
        else:
            status = f'sem conexao em {self.leitor.porta} — reconectando… {self.leitor.erro}'
        self.tela.itemconfig(self.id_status, text=status)

        self.raiz.after(INTERVALO_TELA_MS, self.atualizar)

    def sair(self):
        self.leitor.parar()
        self.raiz.destroy()

    def rodar(self):
        self.raiz.mainloop()


def main():
    porta = descobrir_porta()
    print(f'Lendo telemetria de {porta} @ {BAUD_RATE}. ESC fecha a janela.')
    leitor = LeitorSerial(porta)
    leitor.start()
    Painel(leitor).rodar()


if __name__ == '__main__':
    main()
