(require 'atomic-chrome)
(atomic-chrome-start-server)

(setq atomic-chrome-url-major-mode-alist
      '(("Overleaf" . tex-mode)
      '("github\\.com" . gfm-mode)))

(provide 'my_ghosttext)
