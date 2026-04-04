(function () {
    'use strict';

    // ===== Theme Toggle =====
    const THEME_KEY = 'theme';
    const root = document.documentElement;

    function getPreferredTheme() {
        const stored = localStorage.getItem(THEME_KEY);
        if (stored) return stored;
        return window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
    }

    function applyTheme(theme) {
        root.dataset.theme = theme;
        localStorage.setItem(THEME_KEY, theme);
    }

    applyTheme(getPreferredTheme());

    document.querySelectorAll('.theme-toggle').forEach(function (btn) {
        btn.addEventListener('click', function () {
            var next = root.dataset.theme === 'dark' ? 'light' : 'dark';
            applyTheme(next);
        });
    });

    // ===== Sticky Header Scroll Effect =====
    var header = document.querySelector('.site-header');
    if (header) {
        window.addEventListener('scroll', function () {
            header.classList.toggle('scrolled', window.scrollY > 40);
        }, { passive: true });
    }

    // ===== Scroll Progress Bar =====
    var progressBar = document.querySelector('.progress-bar');
    if (progressBar) {
        window.addEventListener('scroll', function () {
            var scrollTop = window.scrollY;
            var docHeight = document.documentElement.scrollHeight - window.innerHeight;
            progressBar.style.width = docHeight > 0 ? (scrollTop / docHeight * 100) + '%' : '0%';
        }, { passive: true });
    }

    // ===== Scroll Reveal (Intersection Observer) =====
    var revealTargets = document.querySelectorAll('.project-card, .category, .section, .hero, .project-hero');
    if ('IntersectionObserver' in window && revealTargets.length > 0) {
        var prefersReduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
        if (!prefersReduced) {
            revealTargets.forEach(function (el) { el.classList.add('reveal'); });

            var observer = new IntersectionObserver(function (entries) {
                entries.forEach(function (entry) {
                    if (entry.isIntersecting) {
                        entry.target.classList.add('visible');
                        observer.unobserve(entry.target);
                    }
                });
            }, { threshold: 0.08, rootMargin: '0px 0px -40px 0px' });

            revealTargets.forEach(function (el) { observer.observe(el); });
        }
    }

    // ===== Clickable Project Cards =====
    document.querySelectorAll('.project-card').forEach(function (card) {
        card.addEventListener('click', function (e) {
            var link = card.querySelector('h3 a');
            if (link && !e.target.closest('a')) {
                link.click();
            }
        });
    });

    // ===== 3D Card Tilt Effect =====
    var prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

    if (!prefersReducedMotion) {
        var cards = document.querySelectorAll('.project-card');
        var tiltMax = 8;

        cards.forEach(function (card) {
            card.addEventListener('mousemove', function (e) {
                var rect = card.getBoundingClientRect();
                var x = e.clientX - rect.left;
                var y = e.clientY - rect.top;
                var centerX = rect.width / 2;
                var centerY = rect.height / 2;
                var rotateX = ((y - centerY) / centerY) * -tiltMax;
                var rotateY = ((x - centerX) / centerX) * tiltMax;

                card.style.transform = 'perspective(600px) rotateX(' + rotateX + 'deg) rotateY(' + rotateY + 'deg) scale3d(1.02, 1.02, 1.02)';
                card.style.setProperty('--mouse-x', x + 'px');
                card.style.setProperty('--mouse-y', y + 'px');
            });

            card.addEventListener('mouseleave', function () {
                card.style.transform = '';
                card.style.transition = 'transform 0.5s ease, border-color 0.3s ease, box-shadow 0.3s ease';
                setTimeout(function () {
                    card.style.transition = '';
                }, 500);
            });

            card.addEventListener('mouseenter', function () {
                card.style.transition = 'border-color 0.3s ease, box-shadow 0.3s ease';
            });
        });
    }

    // ===== Cursor Glow Effect =====
    if (!prefersReducedMotion && window.innerWidth > 768) {
        var glow = document.createElement('div');
        glow.classList.add('cursor-glow');
        document.body.appendChild(glow);

        var glowX = 0, glowY = 0;
        var currentX = 0, currentY = 0;

        document.addEventListener('mousemove', function (e) {
            glowX = e.clientX;
            glowY = e.clientY;
        }, { passive: true });

        function animateGlow() {
            currentX += (glowX - currentX) * 0.08;
            currentY += (glowY - currentY) * 0.08;
            glow.style.left = currentX + 'px';
            glow.style.top = currentY + 'px';
            requestAnimationFrame(animateGlow);
        }
        animateGlow();

        document.addEventListener('mouseleave', function () {
            glow.style.opacity = '0';
        });
        document.addEventListener('mouseenter', function () {
            glow.style.opacity = '1';
        });
    }

    // ===== Back to Top Button =====
    var backToTop = document.querySelector('.back-to-top');
    if (backToTop) {
        window.addEventListener('scroll', function () {
            backToTop.classList.toggle('visible', window.scrollY > 400);
        }, { passive: true });

        backToTop.addEventListener('click', function () {
            window.scrollTo({ top: 0, behavior: 'smooth' });
        });
    }
})();
