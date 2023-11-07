import requests
import re
import os


def get_version_tuple(version):
    pattern = r"^.*?(\d+)[\.\_](\d+)[\.\_](\d+).*$"
    match = re.match(pattern, version)
    if not match:
        return tuple()
    major = int(match.group(1))
    minor = int(match.group(2))
    patch = int(match.group(3))
    return (major, minor, patch)

def get_github_releases(org, repo):
    url = f"https://api.github.com/repos/{org}/{repo}/releases"
    response = requests.get(url)
    releases = response.json()
    if "message" in releases:
        # Could be a github error on API limit
        print(releases["message"])
        return None
    # if 'jpeg' in org:
    #     print(releases)
    versions = []
    for release in releases:
        tag_name = release["tag_name"].lower()
        name = release["name"].lower()
        if 'rc' not in name and 'beta' not in name and \
           'rc' not in tag_name and 'beta' not in tag_name:
            versions.append(tag_name)
    # print(versions)
    # Filter out rc versions
    versions_int = [(get_version_tuple(v), v) for v in versions if 'rc' not in v]
    # print(versions_int)
    versions_int.sort(reverse=True)
    return versions_int


def get_url_releases(url):
    folder_url = os.path.dirname(url)
    response = requests.get(folder_url)
    html = response.text
    # print(html)

    version_pattern = r"([\d\.]+)\.tar.\w\w\d?"
    versions = re.findall(version_pattern, html)

    # Filter out rc versions
    versions = [(get_version_tuple(v), v) for v in versions if 'rc' not in v]
    # print(versions)
    versions.sort(reverse=True)
    return versions


def parse_github_url(github_url):
    '''
    Retrieve org, name and version from url
    '''
    github_base_url = r"https?:\/\/github\.com\/([\w\-\_]+)\/([\w\-\_]+)"
    github_url_pattern = github_base_url + r"\/.*?(\d+)\.(\d+)\.(\d+).*"
    github_url_matches = re.match(github_url_pattern, github_url)

    version = tuple()
    if github_url_matches:
        organisation_name = github_url_matches.group(1)
        repository_name = github_url_matches.group(2)
        version = (int(github_url_matches.group(3)), int(github_url_matches.group(4)), int(github_url_matches.group(5)))
    else:
        # we may not be able to extract version
        github_url_pattern = github_base_url + r"(?:\/.*)?"
        github_url_matches = re.match(github_url_pattern, github_url)
        if github_url_matches:
            organisation_name = github_url_matches.group(1)
            repository_name = github_url_matches.group(2)
        else:
            return None
    return (organisation_name, repository_name, version)


def parse_url(url):
    '''
    Retrieve org, name and version from url
    '''
    # version with 3 numbers
    url_pattern = r"(https?:\/\/.+?)(\d+)\.(\d+)\.(\d+).*"
    url_matches = re.match(url_pattern, url)

    version = tuple()
    repository_name = ""
    if url_matches:
        organisation_url = url_matches.group(1)
        version = (int(url_matches.group(2)), int(url_matches.group(3)), int(url_matches.group(4)))
    else:
        # Try version with only 2 numbers 
        url_pattern = r"(https?:\/\/.+?)(\d+)\.(\d+).*"
        url_matches = re.match(url_pattern, url)
        if url_matches:
            organisation_url = url_matches.group(1)
            version = (int(url_matches.group(2)), int(url_matches.group(3)))
        else:
            print('FAILED TO PARSE URL')
            return None
    return (organisation_url, repository_name, version)

def get_cmakefile_urls(cmake_file_path):
    '''
    List all URLs in a CMakelists.txt
    '''
    urls = []
    with open(cmake_file_path, "r") as cmake_file:
        cmake_file_content = cmake_file.read()

        url_pattern = r"URL\s+([^\s]+)\s*"
        url_matches = re.findall(url_pattern, cmake_file_content)
        return url_matches

def get_cmakefile_repositories(cmake_file_path):
    '''
    List all URLs in a CMakelists.txt
    '''
    urls = []
    with open(cmake_file_path, "r") as cmake_file:
        cmake_file_content = cmake_file.read()

        url_pattern = r"GIT_REPOSITORY\s+([^\s]+)\s*GIT_TAG\s+([^\s\)]+)\s*"
        url_matches = re.findall(url_pattern, cmake_file_content)
        return url_matches


# Define CMake file to parse
scriptPath = os.path.dirname(os.path.abspath(__file__))
cmake_file_path = os.path.join(os.path.dirname(scriptPath), "CMakeLists.txt")

urls = get_cmakefile_urls(cmake_file_path)
github_urls = []
other_urls = []
for url in urls:
    if 'github' in url:
        github_urls.append(url)
    else:
        other_urls.append(url)

not_up_to_date = []

to_check_manually = []

for url in github_urls:
    if 'AliceVisionDependencies' in url:
        # AliceVisionDependencies is a particular case that cannot be checked automatically
        to_check_manually.append(url)
        continue
    print(f'\n >> {url}')
    org, repo, version = parse_github_url(url)
    print(f'{org}, {repo}: {version}')
    releases = get_github_releases(org, repo)
    # print(f'releases on github: {releases}')
    if not releases:
        print('No official release on github.')
        to_check_manually.append(url)
        continue
    latest_release = releases[0]
    if version != latest_release[0]:
        not_up_to_date.append((org, repo, version, latest_release))
        print(f'Not up-to-date: latest release on github: {latest_release}')
    else:
        print(f'Up to date with github: {latest_release}')


for url in other_urls:
    print(f'\n >> {url}')
    org, repo, version = parse_url(url)
    print(f'{org}, {repo}: {version}')
    # add something at the end of the url to ensure that the dirname will always use the right folder
    releases = get_url_releases(org + "_")
    # print(f'releases online: {releases}')
    if not releases:
        print('No official release online.')
        to_check_manually.append(url)
        continue
    latest_release = releases[0]
    if version != latest_release[0]:
        not_up_to_date.append((org, repo, version, latest_release))
        print(f'Not up-to-date: latest release online: {latest_release}')
    else:
        print(f'Up to date with online information: {latest_release}')

print("\n\nCheck git repositories:")
git_repositories = get_cmakefile_repositories(cmake_file_path)
for repo, tag in git_repositories:
    print(f'\n >> {repo} {tag}')
    if "github" in repo:
        org, repo, version = parse_github_url(repo)
        version = get_version_tuple(tag)
        print(f'{org}, {repo}: {version}')
        releases = get_github_releases(org, repo)
        # print(f'releases on github: {releases}')
        if not releases:
            print('No official release on github.')
            to_check_manually.append(f'{repo}, {tag}')
            continue
        latest_release = releases[0]
        if version != latest_release[0]:
            not_up_to_date.append((org, repo, version, latest_release))
            print(f'Not up-to-date: latest release on github: {latest_release}')
        else:
            print(f'Up to date with github: {latest_release}')
    else:
        to_check_manually.append(f'{repo}, {tag}')


if not_up_to_date:
    print("\n\nREPOSITORIES TO UPDATE:")
    for org, repo, version, latest_release in not_up_to_date:
        print(f' > {org}/{repo}: {version} => {latest_release[1]}')
else:
    print("\n\nALL REPOSITORIES ARE UP-TO-DATE")


if to_check_manually:
    print("\n\nSome projects cannot be checked automatically, you should verify them manually:")
    for url in to_check_manually:
        print(f' > {url}')

print("\n")

