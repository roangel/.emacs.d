(require 'org)

(setq org-agenda-files (list "~/.org/my-org-files/work.org"
                             "~/.org/my-org-files/personal.org"))

(setq org-agenda-span 10
      org-agenda-start-on-weekday nil
      org-agenda-start-day "-5d")

(setq calendar-week-start-day 1)

(setq org-todo-keywords
      '((sequence "TODO" "LOW-PRIO" "VERIFY" "|" "DONE" "DELEGATED" "CANCELLED")))

(with-eval-after-load 'org
  (setq org-startup-indented t) ; Enable `org-indent-mode' by default
  (add-hook 'org-mode-hook #'visual-line-mode))

(provide 'my_org_mode)
