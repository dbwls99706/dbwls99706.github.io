(function () {
    'use strict';

    var root = document.documentElement;
    var reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

    // ===== Theme Toggle =====
    // `remember` is set only when the visitor presses the toggle, so the default stays dark for everyone else.
    function applyTheme(theme, remember) {
        root.dataset.theme = theme;
        if (remember) {
            try { localStorage.setItem('theme-choice', theme); } catch (e) { /* storage unavailable */ }
        }
        var meta = document.querySelector('meta[name="theme-color"]');
        if (meta) meta.content = theme === 'light' ? '#eceef1' : '#0b0d10';
    }
    applyTheme(root.dataset.theme === 'light' ? 'light' : 'dark', false);

    document.querySelectorAll('.theme-toggle').forEach(function (btn) {
        btn.addEventListener('click', function () {
            var next = root.dataset.theme === 'dark' ? 'light' : 'dark';
            if (!document.startViewTransition || reducedMotion) {
                applyTheme(next, true);
                return;
            }
            // The new theme spreads out from the button that was pressed.
            var rect = btn.getBoundingClientRect();
            var x = rect.left + rect.width / 2;
            var y = rect.top + rect.height / 2;
            var radius = Math.hypot(Math.max(x, window.innerWidth - x), Math.max(y, window.innerHeight - y));
            root.classList.add('theme-switching');
            var transition = document.startViewTransition(function () { applyTheme(next, true); });
            transition.ready.then(function () {
                root.animate(
                    { clipPath: ['circle(0px at ' + x + 'px ' + y + 'px)', 'circle(' + radius + 'px at ' + x + 'px ' + y + 'px)'] },
                    { duration: 560, easing: 'cubic-bezier(0.16, 1, 0.3, 1)', pseudoElement: '::view-transition-new(root)' }
                );
            }).catch(function () { /* transition skipped */ });
            var done = function () { root.classList.remove('theme-switching'); };
            transition.finished.then(done, done);
        });
    });

    // ===== Header state + scroll progress =====
    var header = document.querySelector('.site-header');
    var progressBar = document.querySelector('.progress-bar');
    var backToTop = document.querySelector('.back-to-top');
    var ticking = false;

    function onScroll() {
        ticking = false;
        var y = window.scrollY;
        if (header) header.classList.toggle('scrolled', y > 24);
        if (backToTop) backToTop.classList.toggle('visible', y > 600);
        if (progressBar) {
            var max = document.documentElement.scrollHeight - window.innerHeight;
            progressBar.style.transform = 'scaleX(' + (max > 0 ? Math.min(1, y / max) : 0) + ')';
        }
    }
    window.addEventListener('scroll', function () {
        if (!ticking) { ticking = true; requestAnimationFrame(onScroll); }
    }, { passive: true });
    onScroll();

    // ===== Back to Top =====
    if (backToTop) {
        backToTop.addEventListener('click', function () {
            if (window.__lenis) window.__lenis.scrollTo(0);
            else window.scrollTo({ top: 0, behavior: reducedMotion ? 'auto' : 'smooth' });
        });
    }

    // ===== Back link returns to where the visitor was in the list =====
    var backLink = document.querySelector('.back-link');
    if (backLink && window.__fromSite && history.length > 1) {
        var cameFrom = '';
        try {
            var act = window.navigation && window.navigation.activation;
            cameFrom = new URL(act && act.from ? act.from.url : document.referrer).pathname;
        } catch (e) { /* unknown origin page */ }
        if (cameFrom === '/' || cameFrom === '/index.html') {
            backLink.addEventListener('click', function (e) {
                if (e.metaKey || e.ctrlKey || e.shiftKey || e.altKey || e.button !== 0) return;
                e.preventDefault();
                history.back();
            });
        }
    }

    // ===== Current section in the nav =====
    var navLinks = document.querySelectorAll('.site-nav a[href^="#"]');
    if (navLinks.length && 'IntersectionObserver' in window) {
        var byId = {};
        navLinks.forEach(function (a) { byId[a.getAttribute('href').slice(1)] = a; });
        var spy = new IntersectionObserver(function (entries) {
            entries.forEach(function (entry) {
                var link = byId[entry.target.id];
                if (!link) return;
                if (entry.isIntersecting) {
                    navLinks.forEach(function (a) { a.removeAttribute('aria-current'); });
                    link.setAttribute('aria-current', 'location');
                } else if (link.getAttribute('aria-current')) {
                    link.removeAttribute('aria-current');
                }
            });
        }, { rootMargin: '-45% 0px -50% 0px' });
        Object.keys(byId).forEach(function (id) {
            var el = document.getElementById(id);
            if (el) spy.observe(el);
        });
    }

    // ===== Live GitHub Stars =====
    var repoCards = document.querySelectorAll('[data-repo]');
    if (repoCards.length > 0 && typeof fetch === 'function') {
        fetch('https://api.github.com/users/dbwls99706/repos?per_page=100')
            .then(function (r) { return r.ok ? r.json() : Promise.reject(r.status); })
            .then(function (repos) {
                var byName = {};
                repos.forEach(function (r) { byName[r.name] = r.stargazers_count; });
                repoCards.forEach(function (card) {
                    var name = card.getAttribute('data-repo').split('/').pop();
                    var count = byName[name];
                    var el = card.querySelector('.js-star');
                    if (typeof count === 'number' && el) {
                        el.textContent = count.toLocaleString('ko-KR');
                    }
                });
            })
            .catch(function () { /* keep static fallback */ });
    }

    // ===== Live deadends.dev Stats =====
    var deadendsTargets = document.querySelectorAll('.js-deadends-errors, .js-deadends-domains');
    if (deadendsTargets.length > 0 && typeof fetch === 'function') {
        fetch('https://deadends.dev/api/v1/stats.json')
            .then(function (r) { return r.ok ? r.json() : Promise.reject(r.status); })
            .then(function (s) {
                if (typeof s.total_errors === 'number') {
                    document.querySelectorAll('.js-deadends-errors').forEach(function (el) {
                        el.textContent = s.total_errors.toLocaleString('ko-KR');
                    });
                }
                if (typeof s.total_domains === 'number') {
                    document.querySelectorAll('.js-deadends-domains').forEach(function (el) {
                        el.textContent = s.total_domains.toLocaleString('ko-KR');
                    });
                }
            })
            .catch(function () { /* keep static fallback */ });
    }
})();
