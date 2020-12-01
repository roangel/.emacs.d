(use-package pdf-tools
 :pin manual ;; manually update
 :config
 ;; initialise
 (pdf-tools-install)
 ;; open pdfs scaled to fit page
 (setq-default pdf-view-display-size 'fit-page)
 ;; automatically annotate highlights
 (setq pdf-annot-activate-created-annotations t)
 ;; use normal isearch
 (define-key pdf-view-mode-map (kbd "C-s") 'isearch-forward))

;; (add-hook 'pdf-view-mode-hook (lambda() (linum-mode -1)))

;; setup files ending in “.pdf” to open in pdf-view-mode
(add-to-list 'auto-mode-alist '("\\.pdf\\'" . pdf-view-mode))


(provide 'my_pdf_mode)
